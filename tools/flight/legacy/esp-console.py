import serial,sys,time
f=open(sys.argv[1],'ab',buffering=0)
while True:
    try:
        e=serial.Serial(); e.port='/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_98:A3:16:B8:56:74-if00'; e.baudrate=115200; e.timeout=0.5; e.dtr=False; e.rts=False; e.open()
        while True:
            d=e.read(4096)
            if d: f.write(d)
    except Exception as x:
        f.write(("\n[esplog reopen: %s]\n"%x).encode()); time.sleep(1)
