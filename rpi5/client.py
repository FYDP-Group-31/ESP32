import socket
import time
import threading
from typing import Optional

class RPi_Client:
    def __init__(self, host: str = 'localhost', port: int = 12345) -> None:
        """
        Initialize the RPi Client
        
        Args:
            host (str): Server host address (default: 'localhost')
            port (int): Server port number (default: 12345)
        """
        self.host: str = host
        self.port: int = port
        self.socket: Optional[socket.socket] = None
        self.connected: bool = False
    
    def connect(self) -> bool:
        """
        Connect to the server
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to server at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            self.connected = False
            return False
    
    def send_message(self, message: str) -> Optional[str]:
        """
        Send a message to the server and receive response
        
        Args:
            message (str): Message to send to server
            
        Returns:
            str: Response from server, or None if error
        """
        if not self.connected or not self.socket:
            print("Not connected to server. Call connect() first.")
            return None
        
        try:
            # Send message
            self.socket.send(message.encode('utf-8'))
            
            # Receive response
            response_data: bytes = self.socket.recv(1024)
            response: str = response_data.decode('utf-8')
            
            print(f"Sent: {message}")
            print(f"Received: {response}")
            return response
            
        except Exception as e:
            print(f"Error sending message: {e}")
            self.connected = False
            return None
    
    def disconnect(self) -> None:
        """Disconnect from the server"""
        if self.socket:
            try:
                self.socket.close()
                self.connected = False
                print("Disconnected from server")
            except Exception as e:
                print(f"Error disconnecting: {e}")
    
    def interactive_session(self) -> None:
        """Start an interactive session with the server"""
        if not self.connected:
            print("Not connected to server. Call connect() first.")
            return
        
        print("\nInteractive session started. Type 'quit' to exit.")
        print("Available commands: ping, status, time, quit, or any custom message")
        print("-" * 50)
        
        try:
            while self.connected:
                user_input = input("Enter message: ").strip()
                
                if not user_input:
                    continue
                
                response = self.send_message(user_input)
                
                if user_input.lower() == 'quit':
                    break
                
                if response is None:
                    print("Connection lost. Exiting...")
                    break
                    
                print()  # Add blank line for readability
                
        except KeyboardInterrupt:
            print("\nExiting interactive session...")
        except Exception as e:
            print(f"Error in interactive session: {e}")
        finally:
            self.disconnect()


def demo_client() -> None:
    """Demonstration of client functionality"""
    client = RPi_Client()
    
    # Connect to server
    if not client.connect():
        return
    
    # Send some test messages
    test_messages = [
        "ping",
        "status", 
        "time",
        "Hello, server!",
        "This is a test message"
    ]
    
    print("\n=== Demo: Sending test messages ===")
    for message in test_messages:
        response = client.send_message(message)
        if response is None:
            break
        time.sleep(1)  # Wait between messages
        print()  # Add blank line
    
    # Send quit message
    client.send_message("quit")
    client.disconnect()


def main() -> None:
    """Main function with menu options"""
    print("RPi Client")
    print("1. Demo mode (send predefined messages)")
    print("2. Interactive mode (manual input)")
    print("3. Exit")
    
    while True:
        try:
            choice = input("\nEnter choice (1-3): ").strip()
            
            if choice == '1':
                demo_client()
                break
            elif choice == '2':
                client = RPi_Client()
                if client.connect():
                    client.interactive_session()
                break
            elif choice == '3':
                print("Goodbye!")
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
                
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()