Things to do on RPi:

Disable getty (otherwise uart will not work properly)
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service