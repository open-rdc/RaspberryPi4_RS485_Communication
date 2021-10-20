#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# serial_port_loopback3.py
# Will also work on Pyton2.
# Serial port testing on a RaspberryPi 4B with Bluetooth disabled.

from __future__ import print_function
import serial

test_string = "Testing 1 2 3 4".encode('utf-8')
#test_string = b"Testing 1 2 3 4" ### Will also work

port_list0 = ["/dev/serial0", "/dev/ttyAMA0"]

for port in port_list0:

    try:
        serialPort = serial.Serial(port, 115200, timeout = 2)
        serialPort.flushOutput ()
        serialPort.flushInput () # Sytax may change in new version of python3-serial
        print("Opened port", port, "for testing:")
        bytes_sent = serialPort.write(test_string)
        print ("Sent", bytes_sent, "bytes")
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print ("Received0", len(loopback) , "valid bytes, Serial port", port, "working \n")
        else:
            print ("Received0 incorrect data", loopback, "over Serial port", port, "loopback\n")
        serialPort.close()
    except:
        #except IOError:
        print ("Failed at", port, "\n")


port_list1 = ["/dev/serial0", "/dev/ttyAMA1"]

for port in port_list1:

    try:
        serialPort = serial.Serial(port, 115200, timeout = 2)
        serialPort.flushOutput ()
        serialPort.flushInput () # Sytax may change in new version of python3-serial
        print("Opened port", port, "for testing:")
        bytes_sent = serialPort.write(test_string)
        print ("Sent", bytes_sent, "bytes")
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print ("Received1", len(loopback) , "valid bytes, Serial port", port, "working \n")
        else:
            print ("Received1 incorrect data", loopback, "over Serial port", port, "loopback\n")
        serialPort.close()
    except:
        #except IOError:
        print ("Failed at", port, "\n")

port_list2 = ["/dev/serial0", "/dev/ttyAMA2"]

for port in port_list2:

    try:
        serialPort = serial.Serial(port, 115200, timeout = 2)
        serialPort.flushOutput ()
        serialPort.flushInput () # Sytax may change in new version of python3-serial
        print("Opened port", port, "for testing:")
        bytes_sent = serialPort.write(test_string)
        print ("Sent", bytes_sent, "bytes")
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print ("Received2", len(loopback) , "valid bytes, Serial port", port, "working \n")
        else:
            print ("Received2 incorrect data", loopback, "over Serial port", port, "loopback\n")
        serialPort.close()
    except:
        #except IOError:
        print ("Failed at", port, "\n")

port_list3 = ["/dev/serial0", "/dev/ttyAMA3"]

for port in port_list3:

    try:
        serialPort = serial.Serial(port, 115200, timeout = 2)
        serialPort.flushOutput ()
        serialPort.flushInput () # Sytax may change in new version of python3-serial
        print("Opened port", port, "for testing:")
        bytes_sent = serialPort.write(test_string)
        print ("Sent", bytes_sent, "bytes")
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print ("Received3", len(loopback) , "valid bytes, Serial port", port, "working \n")
        else:
            print ("Received3 incorrect data", loopback, "over Serial port", port, "loopback\n")
        serialPort.close()
    except:
        #except IOError:
        print ("Failed at", port, "\n")

port_list4 = ["/dev/serial0", "/dev/ttyAMA4"]

for port in port_list4:

    try:
        serialPort = serial.Serial(port, 115200, timeout = 2)
        serialPort.flushOutput ()
        serialPort.flushInput () # Sytax may change in new version of python3-serial
        print("Opened port", port, "for testing:")
        bytes_sent = serialPort.write(test_string)
        print ("Sent", bytes_sent, "bytes")
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print ("Received4", len(loopback) , "valid bytes, Serial port", port, "working \n")
        else:
            print ("Received4 incorrect data", loopback, "over Serial port", port, "loopback\n")
        serialPort.close()
    except:
        #except IOError:
        print ("Failed at", port, "\n")

# That's it!
