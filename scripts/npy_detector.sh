#!/bin/bash

export CKPT="./data/SqueezeSeg/model.ckpt-23000"
export INPATH="./data/test/g_vlp64/*"
export NET="squeezeSeg"
export CLASSES="ng"
export GPUID=0
export RATE=10
export CRF=1
export TEST=9 # DEFAULT test 

if [ $# -eq 0 ]
then
      echo "Usage: ./scripts/train.sh [options]"
      echo " "
      echo "options:"
      echo "-h, --help              Show brief help"
      echo "-gpu                    Set gpu (id | n (cpu))"
      echo "-rate                   Publishing rate"
      echo "-test                   Test to run"
  	  exit 0
fi

while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "Usage: ./scripts/train.sh [options]"
      echo " "
      echo "options:"
      echo "-h, --help              Show brief help"
      echo "-gpu                    Set gpu (id | n (cpu))"
	  echo "-rate                   Publishing rate"
      echo "-test                   Test to run"
  	  exit 0
      ;;
    -gpu)
      export GPUID="$2"
      shift
      shift
      ;;
    -ckpt)
      export CKPT="$2"
      shift
      shift
      ;;
    -inpath)
      export INPATH="$2"
      shift
      shift
      ;;
    -rate)
	  export RATE="$2"
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
   INPATH="./data/test/ng_vlp16/*"
   CLASSES="ng"
   CRF=1
   CKPT="./data/SqueezeSeg/model.ckpt-23000"
elif [ $TEST -eq 2 ]
then 
   NET="squeezeSeg16"
   INPATH="./data/test/g_vlp16/*"
   CLASSES="ext"
   CRF=1
   CKPT="./data/SqueezeSeg/model.ckpt-23000"
elif [ $TEST -eq 3 ]
then
   NET="squeezeSeg16"
   INPATH="./data/test/ng_vlp16/*"
   CLASSES="ng"
   CRF=0
   CKPT="./data/SqueezeSeg/model.ckpt-23000"
elif [ $TEST -eq 4 ]
then
   NET="squeezeSeg16"
   INPATH="./data/test/g_vlp16/*"
   CLASSES="ext"
   CRF=0
   CKPT="./data/SqueezeSeg/model.ckpt-23000"
elif [ $TEST -eq 5 ]
then
   NET="squeezeSeg32"
   INPATH="./data/test/ng_vlp32/*"
   CLASSES="ng"
   CRF=1
   CKPT="./data/SqueezeSeg/model.ckpt-23000"
elif [ $TEST -eq 6 ]
then 
   NET="squeezeSeg32"
   INPATH="./data/test/g_vlp32/*"
   CLASSES="ext"
   CRF=1
   CKPT="./data/SqueezeSeg/model.ckpt-79000"
elif [ $TEST -eq 7 ]
then
   NET="squeezeSeg32"
   INPATH="./data/test/ng_vlp32/*"
   CLASSES="ng"
   CRF=0
   CKPT="./data/SqueezeSeg/model.ckpt-23000"
elif [ $TEST -eq 8 ]
then
   NET="squeezeSeg32"
   INPATH="./data/test/g_vlp32/*"
   CLASSES="ext"
   CRF=0
   CKPT="./data/SqueezeSeg/model.ckpt-71000"
fi

# bash ./scripts/killall.sh
bash ./scripts/quickstart.sh -rviz_cfg ./rviz/default.rviz

python ./src/visualize.py \
	--checkpoint=$CKPT \
	--input_path=$INPATH \
	--gpu=$GPUID \
  --rate=$RATE \
  --net=$NET \
  --classes=$CLASSES \
  --crf=$CRF \
    

