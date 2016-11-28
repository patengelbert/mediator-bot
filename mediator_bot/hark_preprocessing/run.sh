#!/bin/sh

export DEVICE=plughw:2,0
export LOC=../config/kinect_tf.zip

# Run to adjust the systems parameters
rosrun rqt_reconfigure rqt_reconfigure&

echo "Preprocessing module"
echo batchflow preprocessing.n ${DEVICE} ${LOC}
batchflow preprocessing.n ${LOC}
