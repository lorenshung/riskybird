#!/usr/bin/env python3
"""Single line-timestamping reader of a console tty.

    stty -F <console-tty> 115200 raw -echo -hupcl cs8 -cstopb -parenb
    python3 tools/bench/tsreader.py <console-tty> <out-prefix> <seconds> &

Writes the raw byte stream to <out-prefix>.raw.log and a copy with each line
prefixed by the host wall-clock time it arrived (<epoch seconds> <line>) to
<out-prefix>.ts.log, then stops after <seconds>.

Use it in place of `cat <tty> > log` when a loop period has to be measured on
the host rather than taken from the image's own `dt=` field: the two agreeing
is what shows the kernel tick is the rate rb derived. It must be the ONLY
reader of the tty; two readers split the byte stream between them.
The tty must already be configured (raw, 115200) -- this script does not
touch its settings.
"""
import os
import select
import sys
import time

if len(sys.argv) != 4:
    sys.exit(__doc__)
dev, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
fd = os.open(dev, os.O_RDONLY | os.O_NOCTTY)
raw = open(out + '.raw.log', 'wb'); ts = open(out + '.ts.log', 'w')
end = time.time() + secs; buf = b''
while time.time() < end:
    r, _, _ = select.select([fd], [], [], 0.5)
    if not r: continue
    d = os.read(fd, 4096)
    if not d: continue
    now = time.time(); raw.write(d); raw.flush(); buf += d
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        ts.write('%.3f %s\n' % (now, line.decode('ascii', 'replace').rstrip('\r'))); ts.flush()
if buf: ts.write('%.3f %s\n' % (time.time(), buf.decode('ascii','replace')))
