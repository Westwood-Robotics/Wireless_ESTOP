#!usr/bin/env python
__author__ = "sjj"
__email__ = "jshen@westwoodrobotics.io"
__copyright__ = "Copyright 2021~2025 Westwood Robotics"
__date__ = "May 28, 2025"

__version__ = "2.1.1"
__status__ = "Production" 

'''
Script for ESTOP channel setup
'''

import time
import serial
from termcolor import cprint


class ESTOPManager(object):
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        # serial communication
        self.port     = port
        self.baudrate = baudrate
        self.ser      = None
        self.open_port()

        # data from estop
        self.data4estop = [0xFF, 0x00, 0x00, 0x00, 0xFE]
        self.data4estop_length   = len(self.data4estop)
        self.data4estop_length_1 = self.data4estop_length - 1

        # data to estop
        self.data2estop = [0xFF, 0x00, 0x00, 0x00, 0xFE]

    def open_port(self):
        """
        Open the serial port
        """
        self.ser = serial.Serial(self.port,
                                 self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0)

    def close_port(self):
        """
        Close the serial port
        """
        if self.ser:
            self.ser.close()

    def read_data(self):
        """
        Read data from ESTOP
        """
        while self.ser.in_waiting >= self.data4estop_length:
            if self.ser.read(1)[0] == 0xFF:
                if self.ser.in_waiting >= self.data4estop_length_1:
                    for idx in range(1, self.data4estop_length):
                        self.data4estop[idx] = self.ser.read(1)[0]

                    self.ser.reset_input_buffer()

                    if self.data4estop[-1] == 0xFE:
                        return True
                    else:
                        return False
        return False

    def send_data(self):
        """
        Send data to ESTOP
        """
        self.ser.write(self.data2estop)

def setup_channel():
    em.read_data()
    if em.data4estop[1] != 0x0B:
        print('ESTOP connecting.' +  '.' * i + ' error')
        exit()
    else:
        print('ESTOP connecting.' +  '.' * i + ' connected')
        print('Present Channel Address:', em.data4estop[2])
        print('Present Channel Frequency:', em.data4estop[3])

    channel_addresss  = input('Input new channel address (0~255): ')
    channel_frequency = input('Input new channel frequency (0~255): ')

    confirm = input('Save channel info? (y/n) ')
    if confirm != 'y':
        print('Exit without saving.')
        exit()

    em.data2estop[1] = 0x0C
    em.data2estop[2] = int(channel_addresss)
    em.data2estop[3] = int(channel_frequency)
    em.send_data()


if __name__ == '__main__':
    i = 0
    while 1:
        try:
            em = ESTOPManager(port='/dev/serial/by-id/usb-Raspberry_Pi_Pico_E662608797263229-if00', baudrate=9600)
            em.ser.reset_input_buffer()

            em.data2estop[1] = 0x0A
            while em.ser.in_waiting < em.data4estop_length:
                print('ESTOP connecting.' +  '.' * i, end='\r')
                em.send_data()
                time.sleep(1)
                i += 1
            break
        except:
            print('ESTOP connecting.' +  '.' * i, end='\r')
            i += 1
        
        time.sleep(1)

    try:
        setup_channel()
    except Exception as e:
        cprint("Error: " + str(e), 'red')
    finally:
        em.close_port()