#!/bin/bash

export GPUID=0
export NET="squeezeSeg"
export IMAGE_SET="val"
export LOG_DIR="./log/"
export DATA_DIR="./data/"
export CLASSES="ng"
export RES=0
export TEST=1
export STEPS=80000
export CRF=0
export CKPT=1000

if [ $# -eq 0 ]
then
  echo "Usage: ./scripts/eval.sh [options]"
  echo " "
  echo "options:"
  echo "-h, --help                show brief help"
  echo "-gpu                      gpu id | n (cpu)"
  echo "-image_set                (train|val)"
  echo "-restore                  Train from checkpoint."
  echo "-steps                    Max number of steps used during training."
  echo "-ckpt                     Checkpoint step"
  exit 0
fi

while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "Usage: ./scripts/eval.sh [options]"
      echo " "
      echo "options:"
      echo "-h, --help                show brief help"
      echo "-gpu                      gpu id | n (cpu)"
      echo "-image_set                (train|val)"
      echo "-restore                  Train from checkpoint."
      echo "-steps                    Max number of steps used during training."
      echo "-ckpt                     Checkpoint step"
      exit 0
      ;;
    -gpu)
      export GPUID="$2"
      shift
      shift
      ;;
    -image_set)
      export IMAGE_SET="$2"
      shift
      shift
      ;;
    -restore)
      export RES="$2"
      shift
      shift
      ;;
    -steps)
      export MAX_STEPS="$2"
      shift
      shift
      ;;
    -ckpt)
      export CKPT="$2"
      shift
      shift
      ;;
    -test)
      export TEST="$2"
      shift
      shift
     ;;
    *)
      break
      ;;
  esac
done

# Test 1 - VLP 16, CRF, classes: unknown, car, pedestrian and cyclist
# Test 2 - VLP 16, CRF, classes: unknown, car, pedestrian, cyclist and ground
# Test 3 - VLP 16, no CRF, classes: unknown, car, pedestrian and cyclist 
# Test 4 - VLP 16, no CRF, classes: unknown, car, pedestrian, cyclist and ground
# Test 5 - VLP 32, CRF, classes: unknown, car, pedestrian and cyclist
# Test 6 - VLP 32, CRF, classes: unknown, car, pedestrian, cyclist and ground
# Test 7 - VLP 32, no CRF, classes: unknown, car, pedestrian and cyclist 
# Test 8 - VLP 32, no CRF, classes: unknown, car, pedestrian, cyclist and ground

if [ $TEST -eq 1 ]
then
   NET="squeezeSeg16"
   LOG_DIR="./log/log16_NG_CRF"
   DATA_DIR="./data/ng_vlp16"
   CLASSES="ng"
   CRF=1
elif [ $TEST -eq 2 ]
then 
   NET="squeezeSeg16"
   LOG_DIR="./log/log16_G_CRF"
   DATA_DIR="./data/g_vlp16"
   CLASSES="ext"
   CRF=1
elif [ $TEST -eq 3 ]
then
   NET="squeezeSeg16"
   LOG_DIR="./log/log16_NG_NOCRF"
   DATA_DIR="./data/ng_vlp16"
   CLASSES="ng"
   CRF=0
elif [ $TEST -eq 4 ]
then
   NET="squeezeSeg16"
   LOG_DIR="./log/log16_G_NOCRF"
   DATA_DIR="./data/g_vlp16"
   CLASSES="ext"
   CRF=0
elif [ $TEST -eq 5 ]
then
   NET="squeezeSeg32"
   LOG_DIR="./log/log32_NG_CRF"
   DATA_DIR="./data/ng_vlp32"
   CLASSES="ng"
   CRF=1
elif [ $TEST -eq 6 ]
then 
   NET="squeezeSeg32"
   LOG_DIR="./log/log32_G_CRF"
   DATA_DIR="./data/g_vlp32"
   CLASSES="ext"
   CRF=1
elif [ $TEST -eq 7 ]
then
   NET="squeezeSeg32"
   LOG_DIR="./log/log32_NG_NOCRF"
   DATA_DIR="./data/ng_vlp32"
   CLASSES="ng"
   CRF=0
elif [ $TEST -eq 8 ]
then
   NET="squeezeSeg32"
   LOG_DIR="./log/log32_G_NOCRF"
   DATA_DIR="./data/g_vlp32"
   CLASSES="ext"
   CRF=0
fi


logdir="$LOG_DIR"
traindir="$logdir/train/"
valdir="$logdir/eval_$IMAGE_SET"
filen="checkpoint"

python ./src/eval.py \
   --dataset=KITTI \
   --data_path=$DATA_DIR/ \
   --image_set=$IMAGE_SET \
   --eval_dir="$valdir" \
   --checkpoint_path="$traindir" \
   --net=$NET \
   --gpu=$GPUID \
   --classes=$CLASSES \
   --restore=$RES \
   --max_steps=$STEPS \
   --ckpt_step=$CKPT \
   --crf=$CRF 
