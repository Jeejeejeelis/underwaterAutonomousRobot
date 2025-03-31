#!/usr/bin/python

import serial
import time

ser = serial.Serial("/dev/ttyUSB2", 115200)
rec_buff = ''

def send_at(command, back, timeout):
    rec_buff = ''
    ser.write((command + '\r\n').encode())
    time.sleep(timeout)
    if ser.in_waiting:
        time.sleep(0.01)
        rec_buff = ser.read(ser.in_waiting())
    if back not in rec_buff.decode():
        print(command + ' ERROR')
        print(command + ' back:\t' + rec_buff.decode())
        return 0
    else:
        print(rec_buff.decode())
        return 1

def get_gps_position():
    rec_null = True
    answer = 0
    print('Start GPS session...')
    send_at('AT+CGPS=0', 'OK', 1)
    send_at('AT+CGPS=1', 'OK', 1)
    time.sleep(2)
    while rec_null:
        answer = send_at('AT+CGPSINFO', '+CGPSINFO: ', 1)
        if answer == 1:
            if ',,,,,,' in rec_buff.decode():
                print('GPS is not ready')
                rec_null = False
                time.sleep(1)
            else:
                print('GPS data:', rec_buff.decode())
                rec_null = False
        else:
            print('Error:', answer)
            send_at('AT+CGPS=0', 'OK', 1)
            return False
        time.sleep(1.5)

try:
    get_gps_position()
except Exception as e:
    print('Exception:', e)
finally:
    ser.close()