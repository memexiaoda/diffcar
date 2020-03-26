import serial
from serial import  SerialException 
TEMPADD='B40300010001CFAF'
ser=serial.Serial('/dev/ttyUSB0',9600,timeout=0.01)
buf=TEMPADD.decode("hex")
ser.write(buf)
msg=ser.read(7)

print '1111'
s= (ord(msg[3])*256+ord(msg[4]))*0.1
print s
        
        