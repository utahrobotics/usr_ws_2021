import serial, time
from enum import Enum
import struct
import threading

COM = "COM10"
arduino = serial.Serial(COM, 115200, timeout=.1)

int_to_four_bytes = struct.Struct('<I').pack

class Command(Enum):
	init_all = 1
	align_all = 2
	align_one = 3
	stop_all = 4
	stop_one = 5
	blink_led = 6
	home_port = 7
	cancel = 8

def encodeAlignCommand(fl, fr, bl, br):
    # cmd = motor<<6 | dir<<5 | steps;
    # return cmd
	fl1, fl2, fl3, fl4 = int_to_four_bytes(fl & 0xFFFFFFFF)
	fr1, fr2, fr3, fr4 = int_to_four_bytes(fr & 0xFFFFFFFF)
	bl1, bl2, bl3, bl4 = int_to_four_bytes(bl & 0xFFFFFFFF)
	br1, br2, br3, br4 = int_to_four_bytes(br & 0xFFFFFFFF)
	return bytearray([Command.align_all.value, int(fl4), int(fl3), int(fl2), int(fl1), int(fr4), int(fr3), int(fr2), int(fr1), int(bl4), int(bl3), int(bl2), int(bl1), int(br4), int(br3), int(br2), int(br1)])

def encodeBlink(num_blinks):
	return bytearray([Command.blink_led.value, num_blinks])

def encodeCancel():
	return bytearray([Command.cancel.value])

def encodeHome(port):
	return bytearray([Command.home_port.value, port])

def encodeInit():
	return bytearray([Command.init_all.value])

def readSerial():
	while True:
		data = arduino.read()
		if data:
			print(data.decode(), end = "") #strip out the new lines for now

time.sleep(1) #give the connection a second to settle

read_thread = threading.Thread(target=readSerial)
read_thread.setDaemon(True) 
read_thread.start()

while True:
	user_cmd = input("\nEnter a command:\n")
	
	if user_cmd == "init":
		cmd = encodeInit()
	
	elif user_cmd == "align all":
		left_front = int(input("left front angle:"))
		right_front = int(input("right front angle:"))
		left_back = int(input("left back angle:"))
		right_back = int(input("right back angle:"))
		cmd = encodeAlignCommand(left_front, right_front, left_back, right_back)

	elif user_cmd == "home":
		home_to_use = int(input("home:"))
		cmd = encodeHome(home_to_use)
	
	elif user_cmd == "blink":
		cmd = encodeBlink(4)

	elif user_cmd == "cancel":
		cmd = encodeCancel()

	else:
		cmd = encodeCancel()

	arduino.write(cmd)
	time.sleep(0.1)