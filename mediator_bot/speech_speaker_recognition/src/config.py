import numpy as np

# Change this to config file
FRAMELENGTH = 512
SAMPLERATE = 16000
DATATYPE = np.int16
MINTRANSCRIBELENGTH = 1  # Note all transcriptions are billed in intervals of 15s
MINDIARIZELENGTH = 1
MAXREQUESTS = 14400 + 50000  # Free + Trial
RPCPERIOD = 0.5
TIMEOUT = 10
MAXCONCURRENT = 4
SPEAKERMODEL = "test.bin"

DISCOVERY_URL = ('https://{api}.googleapis.com/$discovery/rest?'
                 'version={apiVersion}')

SAVELOADEVENTSUPPORTED = False