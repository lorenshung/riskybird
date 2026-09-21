#!/bin/bash
D=/scratch2/dima/misc_sw/gtsrb_signclf
PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python
rm -f $D/logs/train.done
setsid bash -c "cd $D && $PY train.py --epochs 35 > $D/logs/train.log 2>&1; echo EXIT_\$? > $D/logs/train.done" < /dev/null > /dev/null 2>&1 &
echo "launched train pid-detached"
