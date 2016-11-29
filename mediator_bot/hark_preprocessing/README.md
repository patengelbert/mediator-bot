# Audio preprocessing module

Based on HARK. http://www.hark.jp/

This module receives a multi channel audio stream from a Kinect and separates the stream into the number of sources specfied. 

The data is outputed on the following topics:

- /HarkWave : the altered 4 channel audio stream
- /HarkSource : the position of the different sources
- /HarkSrcWave : the separated audio with its source direction

## Use
