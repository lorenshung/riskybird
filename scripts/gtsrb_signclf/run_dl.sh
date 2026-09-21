#!/bin/bash
cd /scratch2/dima/misc_sw/gtsrb_signclf
export PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python
setsid bash -c '/scratch2/dima/miniforge3/envs/xpurt/bin/python /scratch2/dima/misc_sw/gtsrb_signclf/dl_gtsrb.py > /scratch2/dima/misc_sw/gtsrb_signclf/logs/dl.log 2>&1; echo EXIT_$? >> /scratch2/dima/misc_sw/gtsrb_signclf/logs/dl.log' < /dev/null > /dev/null 2>&1 &
echo "launched dl"
