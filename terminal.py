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
            #ser.write(b'Testing: Hello there!\r\n')
        except SerialException:
            if ((t:=time()) - last_time >= 5):
                print('NOTICE: Unable to connect, retrying...')
                last_time = t
    
    else:
        try:
            value = ser.readline()
            line = value.decode('ascii').strip('\r\n ') # don't want to strip tabs
            if line == 'Inside main loop':
                #print('NOTICE: Sending "Hello there"')
                #ser.write(b'Hello there!\r\n')
                print('NOTICE: sending message')
                ser.write(bytes([0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x01, 0x06, 0x07, 0x69, 0x00]))
            print(line)
        except SerialException:
            print('NOTICE: Lost connection')
            ser.is_open = False

        except UnicodeDecodeError:
            #print('Got non-ascii text:')
            print(value)