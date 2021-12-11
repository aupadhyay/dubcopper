file = ReadSensors

default:
	arduino-cli compile --fqbn arduino:mbed_nano:nano33ble $(file) && arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino:mbed_nano:nano33ble $(file)
build:
	arduino-cli compile --fqbn arduino:mbed_nano:nano33ble $(file)
upload:
	arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino:mbed_nano:nano33ble $(file)
boards:
	arduino-cli board list
serial:
	stty -f /dev/cu.usbmodem2101 raw 115200
	cat /dev/cu.usbmodem2101