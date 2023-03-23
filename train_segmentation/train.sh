#!/bin/bash

# Load the JSON configuration file
CONFIG_FILE=$1
CONFIG=$(cat $CONFIG_FILE)

# Parse the JSON configuration
MODEl_TYPE=$(echo $CONFIG | jq -r '.model_type')
EPOCHS=$(echo $CONFIG | jq -r '.epochs')
BATCH_SIZE=$(echo $CONFIG | jq -r '.batch_size')
LEARNING_RATE=$(echo $CONFIG | jq -r '.learning_rate')
LOAD=$(echo $CONFIG | jq -r '.load')
SCALE=$(echo $CONFIG | jq -r '.scale')
VALIDATION=$(echo $CONFIG | jq -r '.validation')
AMP=$(echo $CONFIG | jq -r '.amp')
BILINEAR=$(echo $CONFIG | jq -r '.bilinear')
CLASSES=$(echo $CONFIG | jq -r '.classes')
MODEL_TYPE=$(echo $CONFIG | jq -r '.model_type')

# Construct the command string
CMD="python3 Pytorch-segmentation-offline/train.py"
if [ ! -z "$EPOCHS" ]; then CMD+=" --epochs $EPOCHS"; fi
if [ ! -z "$BATCH_SIZE" ]; then CMD+=" --batch-size $BATCH_SIZE"; fi
if [ ! -z "$LEARNING_RATE" ]; then CMD+=" --learning-rate $LEARNING_RATE"; fi
if [ ! -z "$LOAD" ]; then CMD+=" --load $LOAD"; fi
if [ ! -z "$SCALE" ]; then CMD+=" --scale $SCALE"; fi
if [ ! -z "$VALIDATION" ]; then CMD+=" --validation $VALIDATION"; fi
if [ ! -z "$AMP" ]; then CMD+=" --amp"; fi
if [ ! -z "$BILINEAR" ]; then CMD+=" --bilinear"; fi
if [ ! -z "$CLASSES" ]; then CMD+=" --classes $CLASSES"; fi
if [ ! -z "$MODEL_TYPE" ]; then CMD+=" --model_type $MODEl_TYPE"; fi

# Execute the command
echo $CMD
eval $CMD
