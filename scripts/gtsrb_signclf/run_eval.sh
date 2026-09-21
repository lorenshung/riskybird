#!/bin/bash
D=/scratch2/dima/misc_sw/gtsrb_signclf
PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python
rm -f $D/logs/eval.done
setsid bash -c "cd $D && \
  $PY eval_int8.py --channels 1 --n_calib 128 > $D/logs/eval_gray.log 2>&1; \
  $PY eval_int8.py --channels 3 --n_calib 128 > $D/logs/eval_rgb.log 2>&1; \
  echo EXIT_\$? > $D/logs/eval.done" < /dev/null > /dev/null 2>&1 &
echo "launched eval (gray then rgb)"
