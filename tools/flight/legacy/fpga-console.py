import serial,sys,time
s=serial.Serial('/dev/ttyUSB9',115200,timeout=0.5)
f=open(sys.argv[1],'ab',buffering=0)
while True:
    d=s.read(4096)
    if d: f.write(d)
