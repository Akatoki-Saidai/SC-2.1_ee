import serial
import time

ser = serial.Serial("/dev/cu.usbserial-1410",115200,timeout=None)
time.sleep(3)
f = open('ultra.csv', 'a')
f.write("num\n")

while True:
	c = ser.read()
	if c == b'1':
		f.write("1")
	elif c == b'2':
		f.write("2")
	elif c == b'3':
		f.write("3")
	elif c == b'4':
		f.write("4")
	elif c == b'5':
		f.write("5")
	elif c == b'6':
		f.write("6")
	elif c == b'7':
		f.write("7")
	elif c == b'8':
		f.write("8")
	elif c == b'9':
		f.write("9")
	elif c == b'0':
		f.write("0")
	elif c == b'\r':
		continue
	elif c == b'\n':
		f.write("\n")
	else:
		continue