import serial

ser = serial.Serial(
    port='/dev/cu.usbmodem54269801',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)
while(1):
	# if SERIAL:
	x_send = 3
	y_send = 4
	MESSAGE = [str(x_send).zfill(3),str(y_send).zfill(3),'\n']
	# history.append(MESSAGE)
	MESSAGE = ''.join(MESSAGE)
	ser.write(MESSAGE.encode("utf-8"))
	# ser.write(MESSAGEstr.encode(chr(x_send)))