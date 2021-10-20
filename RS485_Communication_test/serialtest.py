#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# serial_port_loopback3.py
# Will also work on Pyton2.
# Serial port testing on a RaspberryPi 4B with Bluetooth disabled.

from __future__ import print_function
import serial

test_string = "Testing 1 2 3 4".encode('utf-8')
#test_string = b"Testing 1 2 3 4" ### Will also work

port_list = ["/dev/serial0", "/dev/ttyAMA0"]

for port in port_list:

    try:
        serialPort = serial.Serial(port, 115200, timeout = 2)
        serialPort.flushOutput ()
        serialPort.flushInput () # Sytax may change in new version of python3-serial
        print("Opened port", port, "for testing:")
        bytes_sent = serialPort.write(test_string)
        print ("Sent", bytes_sent, "bytes")
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print ("Received", len(loopback) , "valid bytes, Serial port", port, "working \n")
        else:
            print ("Received incorrect data", loopback, "over Serial port", port, "loopback\n")
        serialPort.close()
    except:
        #except IOError:
        print ("Failed at", port, "\n")

# That's it!
