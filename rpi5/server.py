import socket
import threading
import time
import logging
from typing import List, Tuple, Optional

class RPi_Server:
    def __init__(self, host: str = 'localhost', port: int = 12345) -> None:
        """
        Initialize the RPi Server
        
        Args:
            host (str): Server host address (default: 'localhost')
            port (int): Server port number (default: 12345)
        """
        self.host: str = host
        self.port: int = port
        self.socket: Optional[socket.socket] = None
        self.is_running: bool = False
        self.clients: List[socket.socket] = []
        self.client_threads: List[threading.Thread] = []
        
        # Setup logging
        logging.basicConfig(level=logging.INFO, 
                          format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
    
    def start_server(self) -> None:
        """Start the server and begin listening for connections"""
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to address and port
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)  # Allow up to 5 pending connections
            
            self.is_running = True
            self.logger.info(f"Server started on {self.host}:{self.port}")
            
            # Accept connections in a loop
            while self.is_running:
                try:
                    client_socket, client_address = self.socket.accept()
                    self.logger.info(f"Client connected from {client_address}")
                    
                    # Create a new thread for each client
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, client_address)
                    )
                    client_thread.start()
                    
                    self.clients.append(client_socket)
                    self.client_threads.append(client_thread)
                    
                except socket.error as e:
                    if self.is_running:
                        self.logger.error(f"Error accepting connection: {e}")
                    break
                    
        except Exception as e:
            self.logger.error(f"Server error: {e}")
        finally:
            self.stop_server()
    
    def handle_client(self, client_socket: socket.socket, client_address: Tuple[str, int]) -> None:
        """
        Handle communication with a connected client
        
        Args:
            client_socket: The client's socket connection
            client_address: The client's address tuple
        """
        try:
            while self.is_running:
                # Receive data from client
                data: bytes = client_socket.recv(1024)
                if not data:
                    break
                
                message: str = data.decode('utf-8').strip()
                self.logger.info(f"Received from {client_address}: {message}")
                
                # Process the message and send response
                response: str = self.process_message(message)
                client_socket.send(response.encode('utf-8'))
                
        except ConnectionResetError:
            self.logger.info(f"Client {client_address} disconnected")
        except Exception as e:
            self.logger.error(f"Error handling client {client_address}: {e}")
        finally:
            # Clean up client connection
            try:
                client_socket.close()
                if client_socket in self.clients:
                    self.clients.remove(client_socket)
            except:
                pass
            self.logger.info(f"Client {client_address} connection closed")
    
    def process_message(self, message: str) -> str:
        """
        Process incoming messages from clients and generate responses
        
        Args:
            message (str): The message received from client
            
        Returns:
            str: Response message to send back to client
        """
        # Basic message processing - can be extended based on your needs
        if message.lower() == "ping":
            return "pong"
        elif message.lower() == "status":
            return f"Server running with {len(self.clients)} connected clients"
        elif message.lower() == "time":
            return f"Server time: {time.strftime('%Y-%m-%d %H:%M:%S')}"
        elif message.lower() == "quit":
            return "Goodbye!"
        else:
            return f"Echo: {message}"
    
    def broadcast_message(self, message: str) -> None:
        """
        Send a message to all connected clients
        
        Args:
            message (str): Message to broadcast
        """
        disconnected_clients: List[socket.socket] = []
        
        for client in self.clients:
            try:
                client.send(message.encode('utf-8'))
            except:
                disconnected_clients.append(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            if client in self.clients:
                self.clients.remove(client)
    
    def stop_server(self) -> None:
        """Stop the server and close all connections"""
        self.logger.info("Stopping server...")
        self.is_running = False
        
        # Close all client connections
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        
        # Close server socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        # Wait for all client threads to finish
        for thread in self.client_threads:
            thread.join(timeout=1.0)
        
        self.logger.info("Server stopped")
    
    def get_connected_clients(self) -> int:
        """Return the number of connected clients"""
        return len(self.clients)


def main() -> None:
    """Main function to start the server"""
    server = RPi_Server()
    
    try:
        server.start_server()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.stop_server()


if __name__ == "__main__":
    main()