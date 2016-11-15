##Speaker Recognition
Microsoft Cognitive Services: ~7s for 5s file -> ~12s delay. Fails at 1s audio
WISS - Written in MAtlab and not automatically compatible with Ros
speaker-recognitionA ~0.45s for 1s file ~0.5s for 5s file. High accuracy and local
PyCasp - Requires Nvidia GPU
Alize - Issues with installation

Final choice: speaker-recognition

##Speech Transcription
Google Speech - ~0.5s for 1 word ~1s for 1 phrase
Microsoft Speech: ~0.68s for 1 word, ~0.7s for 1 phrase. Intent not available for python -> may be useful to use text analysis afterwards
PocketSphinx: ~2s for 1 word, also gets it wrong
wit.ai: ~2s for 1 word, same for 1 phrase free though
IBM Watson: Weird subscription method

Final choice: Microsoft speech/goolge speech + microsoft intent analysis
