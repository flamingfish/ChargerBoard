from serial import Serial, SerialException
from time import time

port = input('What is the PORT number?\n')

ser = Serial()
ser.port = f'COM{port}'
ser.baudrate = 9600

last_time = time()

while 1:
    if not ser.is_open:
        try:
            ser.open()
            print('NOTICE: Established connection')
        except SerialException:
            if ((t:=time()) - last_time >= 5):
                print('Unable to connect, retrying...')
                last_time = t
    
    else:
        try:
            value = ser.readline()
            line = value.decode('ascii').strip()
            print(line)
        except SerialException:
            print('NOTICE: Lost connection')
            ser.is_open = False

        except UnicodeDecodeError:
            print('Got non-ascii text:')
            print(value)