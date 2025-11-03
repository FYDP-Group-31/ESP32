# Setup instructions
1. Run `source ./setup.sh`. This needs to be run every time a new terminal environment is used.
2. `ls /dev` and find the port the ESP32 is connected to

# Build
1. `idf.py build`

# Flash
1. `idf.py -p <PORT> flash`

Example: `idf.py -p /dev/cu.usbmodem101 flash`

# Monitor outputs
1. `idf.py -p <PORT> monitor`

Example: `idf.py -p /dev/cu.usbmodem101 monitor`

# Clean
1. To remove build files: `idf.py clean`
2. To remove build files and esp-idf components: `idf.py fullclean`

# Shortcuts
All idf.py commands can be combined into a single line

Example: `idf.py -p <PORT> build flash monitor`
