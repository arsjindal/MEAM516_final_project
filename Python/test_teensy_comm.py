import serial
import time
import numpy as np

ser = serial.Serial(
    port='/dev/ttyACM1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)
def SendPixelValues(x_send,y_send):
	MESSAGE = [str(x_send).zfill(3),str(y_send).zfill(3),'\n']
	MESSAGE = ''.join(MESSAGE)
	ser.write(MESSAGE.encode("utf-8"))

def MoveToSmooth(x_start, y_start, x_end,y_end):
	x = np.linspace(x_start,x_end,num=10)
	y = np.linspace(y_start,y_end,num=10)

	for i_x, i_y in zip(x, y):
		SendPixelValues(int(i_x),int(i_y))



while(1):
	MoveToSmooth(0,0,0,255)
	MoveToSmooth(0,255,255,255)
	MoveToSmooth(255,255,255,0)
	MoveToSmooth(255,0,0,0)
	# SendPixelValues(255,127)
	# time.sleep(0.1)

	# SendPixelValues(0,127)
	# time.sleep(0.1)

	# SendPixelValues(255,255)
	# time.sleep(1)

	# SendPixelValues(255,0)
	# time.sleep(1)