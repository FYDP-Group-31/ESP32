# Things to do on RPi:

Disable getty (otherwise uart will not work properly)

```sudo systemctl stop serial-getty@ttyAMA0.service```

```sudo systemctl disable serial-getty@ttyAMA0.service```

This ensures that serial login is off.

# Running server
```nohup python rpi5/server.py > server.log 2>&1 &```

Running this will run the socket server in the background.

To kill the process, find the PID via:

```ps aux | grep "python3 rpi5/server.py"```

And do ```kill <PID>```