import serial
import time

class SMS:
    
    def sendSMS1():
        port = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
        port.write(b'AT\r')
        rcv = port.read(10)
        print(rcv)
        time.sleep(1)        
        port.write(b"AT+CMGF=1\r")
        print("Text Mode Enabled…")
        time.sleep(3)
        port.write(b'AT+CMGS="9491500284"\r')
        msg = "WARNING:"+"\n"+"NBSC Smart Bin"+"\n"+"Biodegradable Bin is full"+"\n"+"Ready to Collect!"
        print("sending message….")
        time.sleep(3)
        port.reset_output_buffer()
        time.sleep(1)
        port.write(str.encode(msg+chr(26)))
        time.sleep(3)
        print("message sent…")

    def sendSMS2():
        port = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
        port.write(b'AT\r')
        rcv = port.read(10)
        print(rcv)
        time.sleep(1)        
        port.write(b"AT+CMGF=1\r")
        print("Text Mode Enabled…")
        time.sleep(3)
        port.write(b'AT+CMGS="9491500284"\r')
        msg = "WARNING:"+"\n"+"NBSC Smart-Bin"+"\n"+"Recyclable Bin is full"+"\n"+"Ready to Collect!"
        print("sending message….")
        time.sleep(3)
        port.reset_output_buffer()
        time.sleep(1)
        port.write(str.encode(msg+chr(26)))
        time.sleep(3)
        print("message sent…")

    def sendSMS3():
        port = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
        port.write(b'AT\r')
        rcv = port.read(10)
        print(rcv)
        time.sleep(1)        
        port.write(b"AT+CMGF=1\r")
        print("Text Mode Enabled…")
        time.sleep(3)
        port.write(b'AT+CMGS="9491500284"\r')
        msg = "WARNING:"+"\n"+"NBSC Smart Bin"+"\n"+"Non-Biodegradable Bin is full"+"\n"+"Ready to Collect!"
        print("sending message….")
        time.sleep(3)
        port.reset_output_buffer()
        time.sleep(1)
        port.write(str.encode(msg+chr(26)))
        time.sleep(3)
        print("message sent…")
