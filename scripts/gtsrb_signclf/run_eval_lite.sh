#!/bin/bash
D=/scratch2/dima/misc_sw/gtsrb_signclf
PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python
rm -f $D/logs/eval_lite.done
setsid bash -c "cd $D && \
  $PY eval_lite.py --channels 1 --n_calib 128 > $D/logs/eval_lite_gray.log 2>&1; \
  $PY eval_lite.py --channels 3 --n_calib 128 > $D/logs/eval_lite_rgb.log 2>&1; \
  echo EXIT_\$? > $D/logs/eval_lite.done" < /dev/null > /dev/null 2>&1 &
echo "launched lite eval"
