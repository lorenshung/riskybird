#!/usr/bin/env python3
"""Line-timestamp a console log that another process is already writing.

    python3 tools/bench/tsfollow.py <console-log> <out-prefix> <seconds> &

Follows <console-log> from its current end, the way `tail -f` does. It never
opens the tty. New bytes go to <out-prefix>.raw.log. A copy goes to
<out-prefix>.ts.log with each line prefixed by the host wall-clock time it was
read (<epoch seconds> <line>). The script stops after <seconds>. The output
format is the same as tools/bench/tsreader.py.

Use it when a console logger already owns the tty (for example a bench
dashboard's reader appending to a file). Opening the tty a second time would
split the byte stream between two readers and corrupt both. Before trusting
the result, check that the log is live: bytes should appear within seconds of
resuming the core. A logger that has died leaves a frozen file, and following
a frozen file looks exactly like a silent board.

Host timestamps here are the time a line reached the file plus up to 50 ms of
polling, so a loop period measured from them is as good as the other logger's
own buffering.
"""
import sys
import time

if len(sys.argv) != 4:
    sys.exit(__doc__)
src, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
f = open(src, 'rb')
f.seek(0, 2)
raw = open(out + '.raw.log', 'wb')
ts = open(out + '.ts.log', 'w')
end = time.time() + secs
buf = b''
while time.time() < end:
    d = f.read()
    if not d:
        time.sleep(0.05)
        continue
    now = time.time()
    raw.write(d)
    raw.flush()
    buf += d
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        ts.write('%.3f %s\n' % (now, line.decode('ascii', 'replace').rstrip('\r')))
        ts.flush()
if buf:
    ts.write('%.3f %s\n' % (time.time(), buf.decode('ascii', 'replace')))
