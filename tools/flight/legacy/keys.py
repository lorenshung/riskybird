import serial,sys,time
s=serial.Serial(); s.port='/dev/ttyUSB9'; s.baudrate=115200; s.dtr=False; s.rts=False; s.open()
for tok in sys.argv[1:]:
    if tok.startswith('sleep:'): time.sleep(float(tok[6:])); continue
    for ch in tok: s.write(ch.encode()); s.flush(); time.sleep(0.15)
s.close()
