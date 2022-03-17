import serial, time
from enum import Enum

class Command(Enum):
	init_all = 1
	align_all = 2
	align_one = 3
	stop_all = 4
	stop_one = 5
	blink_led = 6
	home_port = 7

def encodeAlignCommand(fl, fr, bl, br):
    # cmd = motor<<6 | dir<<5 | steps;
    # return cmd
    return bytearray([Command.align_all.value, fl, fr, bl, br])

def encodeBlink(num_blinks):
	return bytearray([Command.blink_led.value, num_blinks])

def encodeHome(port):
	return bytearray([Command.home_port.value, port])

def encodeInit():
	return bytearray([Command.init_all.value])


arduino = serial.Serial('COM3', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

while True:
	user_cmd = input("\nEnter a command:\n")
	
	if user_cmd == "init":
		cmd = encodeInit()
	
	if user_cmd == "align all":
		left_front = int(input("left front angle:"))
		right_front = int(input("right front angle:"))
		left_back = int(input("left back angle:"))
		right_back = int(input("right back angle:"))
		cmd = encodeAlignCommand(left_front, right_front, left_back, right_back)

	if user_cmd == "home":
		home_to_use = int(input("home:"))
		cmd = encodeHome(home_to_use)
	
	if user_cmd == "blink":
		cmd = encodeBlink(4)

	arduino.write(cmd)
	data='0'.encode()
	while data != b'\x00':
		data = arduino.read()
		if data:
			print(data.decode(), end = "") #strip out the new lines for now
			# (better to do .read() in the long run for this reason
