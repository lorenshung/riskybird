#!/bin/bash
D=/scratch2/dima/misc_sw/gtsrb_signclf
PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python
rm -f $D/logs/train_lite.done
setsid bash -c "cd $D && $PY train_lite.py --epochs 40 > $D/logs/train_lite.log 2>&1; echo EXIT_\$? > $D/logs/train_lite.done" < /dev/null > /dev/null 2>&1 &
echo "launched lite training"
