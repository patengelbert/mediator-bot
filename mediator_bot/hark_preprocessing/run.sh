#!/bin/sh

PROJECTDIR="$PWD/mediator_bot"
export DEVICE=plughw:1,0
export LOC=$PROJECTDIR/config/kinect_tf.zip

echo $PROJECTDIR

# Run to adjust the systems parameters
rosrun rqt_reconfigure rqt_reconfigure &

echo "Preprocessing module"
echo batchflow "$PROJECTDIR/hark_preprocessing/preprocessing.n" ${LOC} ${DEVICE}
batchflow "$PROJECTDIR/hark_preprocessing/preprocessing.n" ${LOC} ${DEVICE}
