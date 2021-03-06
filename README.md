# Mediator-Bot

[![Build Status](https://travis-ci.com/patengelbert/mediator-bot.svg?token=imo3fxmH6VGoPsvCNfup&branch=master)](https://travis-ci.com/patengelbert/mediator-bot)

The main function of the mediator-bot is to act as a mediator for a conversation/debate/meeting, trying to balance the conversation so that each participant is given the chance to speak.

Using:
* Speech Recognition - voice recognition, determining if multiple people are talking and split into separate channels, produce transcript, determine volume level and tone of conversation
* Processing - determine the balance of the discussion, decide what actions by the robot are required, display visual representation of conversation statistics
* Robot - gestures and speech to direct conversation, interaction (e.g. turn head towards speaker)
* User Interface - display useful stats, display setup options

Possible test - hold two separate meetings, one with a human mediator and one with the mediator-bot, gather participants' opinions on which was preferred.

## Requirements
- Python 2.7
- Make ```sudo apt-get install make```
- Python 2.7 virtualenv ```pip install virtualenv```
- portaudio19-dev ```sudo apt-get install portaudio19-dev```

## Installation
- Run ```make build```
- Enable the virtual environment ```. env/bin/activate```

## Notes
- Add any additional python packages that are required to the requirements.txt
- Add unit tests for all new code to mediator_bot/test
- I (Patrick) will get an email every timer a commit is made to master. I will also know who and what broke it and will chase you up on it.
- Test everything before pushing
- Develop on a local dev branch 
``` [bash]
git branch dev master
git checkout dev
git branch --set-upstream-to=master
```
- Update your dev branch by rebasing from master
``` [bash]
git stash
git checkout master
git pull
git checkout dev
git rebase
git stash pop
```

## Tasks
- Cocktail party problem, preprocessing for speech recognition (Split sound into separate streams for the number of current speakers)
- Speech recognition, transcribing the conversation (Who is speaking, what are the words that they are saying)
- Natural Language processing, interpreting the text derived from the speech recognition (speaker, person spoken to, emotion, etc.)
- AI framework from current mood to appropriate response (link input to correct response)
- Library of actions for nao eg. pointing, waving, signalling to listen
- Library of spoken phrases to use in response to specific situations, plus a wrapper class
- UI for tablet/laptop (who is speaking, speech balance, who is sitting where)

## Reports

[Design Report](https://www.overleaf.com/6835404kgznhmrnwdwq)

[Final Report](https://www.overleaf.com/7427603bmmydhjfsxxn)

[Google Form](https://docs.google.com/forms/d/e/1FAIpQLSdMQ1PWSnZSL3sv7OPfQDM9imTj5WT-ATlbbMTxmg3NjYFt5g/viewform)

## System

![System Overview](https://github.com/patengelbert/mediator-bot/raw/master/images/System Overview.png?raw=true)

## Topics

### SPEAKER + SPEECH RECOGNITION TOPICS
#### Subscribed to
- "HarkSrcWave" topic, sends seperated stream data with azimuth. Msg type: "int":numChannels, "int":azimuth, ```float32[channelNum][frameLength]```
- "Person_setup" topic, sends request to register a new person, Msg type: "string":label, "dict":meta_data(pronoun)
#### Publishes
- "Transcript" topic, sends the transcript of a certain stream, Msg type: timestamp (of input), "int":channel_id, "string":transcript
- "Speaker" topic, sends the label and metadata of a certain stream, Msg type: timestamp(of input), "int":channel_id, "string":label, "dict":metadata
- "Registration_complete" topic, sends the label and registration status, Msg type: "string":label, "bool":success, "int":time_spoken

#### Notes
- Timestamp + channel_id combination of published messages are guaranteed to refer to the same input (use to match a audio stream with speaker or speaker with transcript)
- 
### UI TOPICS
#### Subscribed to
- "Transcript" topic, sends what spoken, by who and when. Msg type: timestamp, "string":persons name, "string": speech spoken
- "Action topic, sent when action occurs or mood changes. Msg type: "string": action type, "string mood": how happy is the system, "string" : person involved (opt) e.g. Sam stop talking over. He's done it 5 times so system mood is sad
- "Percentage" topic, sent person and % they've spoken when changes sig. Msg type: "string":person name, "int" : percentage spoken
- "Register_Person" topic, returns whether person successfully registered to system Msg type: "string": person name, "bool": 0/1 fail/success
#### Publishing to
- "Person_Setup" topic, sends when person trying to register to system

#### System moods
* "Happy"
* "Sad"
* "Angry"
* "Shocked"
* "Neutral"

#### System Actions
* "too_loud"
* "too_quiet"
* "speaking_over"
* "dominated"


##Timeline

| Date          | Deadline          | Deliverables    |
| ------------- |:-----------------| :--------------|
| 3/11/16       | 4 page Design Report     | - [x] Research Hypothesis with stats<br>  - [x] System Design Diagram<br>  - [x] your proposed experimental validation methodology     |
| 17/11/16    | 4 page Individual Component Paper         |  - [x]     Report the state of the subcomponent you are responsible for <br> - [x] Design to be followed for the subcomponent      |
| 15/12/16 | Final Presentation, Live demo          |   - [ ] It Works         |
| 22/12/16 | 8 page Final Report Submission (LaTex)         |   - [ ] Final System Design<br> - [ ] Experimental results  analysis <br> - [ ] Compressed file with all experimental results & readme.md <br> - [ ] Bonus 5% video-Explain hypothesis, experimental findings, potential use cases <br>*Look at examples on BB*       |
## Similar Projects

 - [MIT Mediator Software](http://hd.media.mit.edu/tech-reports/TR-616.pdf)
 - [Ubiquitous Meeting Facilitator with Playful Real-Time User Interface](http://link.springer.com/chapter/10.1007%2F978-3-642-23641-9_3)
 - [Speaker Diarization (PyCASP?)](http://www.icsi.berkeley.edu/pubs/speech/fastspeakerdiarization11.pdf)
 - [Robot Mediator for Classrooms](http://link.springer.com/article/10.1007/s10514-008-9101-z)
 - [Robot Mediator for 3 party conversation](http://www.sciencedirect.com/science/article/pii/S0885230814001260)
 - [Social Dynamics in multiparty conversations](http://onlinelibrary.wiley.com/doi/10.1111/j.1540-4560.1948.tb01783.x/abstract)
 - [Microsoft is doing waht we want to](https://www.microsoft.com/en-us/research/project/meeting-recognition-and-understanding/)
 
## Resources

[Test Meetings](http://groups.inf.ed.ac.uk/ami/download/)

Notes
 - https://www.quora.com/Can-I-do-speaker-recognition-project-with-python
 - https://www.quora.com/What-are-the-best-webcams-for-computer-vision-projects
 - http://cslu.ohsu.edu/~bedricks/courses/cs655/hw/hw4/hw4.html
 - http://stackoverflow.com/questions/7309219/python-speaker-recognition
 - http://stackoverflow.com/questions/20968399/camera-suggestion-for-machine-vision-with-opencv
 - http://robotics.stackexchange.com/questions/1615/why-can-humans-single-out-audio-in-a-crowd-what-would-it-take-for-a-robot-to-do/1625#1625
 - http://digitalcommons.wustl.edu/cgi/viewcontent.cgi?article=1667&context=pacs_capstones
 - https://www.youtube.com/watch?v=ihAG6cMpUlY&index=2&list=PLvOlSehNtuHv98KUcud260yJBRQngBKiw
 - http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.43.139
 - http://www.grc.nasa.gov/WWW/Acoustics/testing/instrumentation/arraymic.htm
 - https://code.google.com/archive/p/voiceid/
 - https://www.google.co.uk/url?sa=t&source=web&rct=j&url=http://www1.icsi.berkeley.edu/~fractor/papers/friedland_64.pdf&ved=0ahUKEwix0MfmjerPAhWMLMAKHUuACuQQFggbMAA&usg=AFQjCNGZtE8DhNTQvzM_Gp3joqhW_CZ4JQ&sig2=evHgr21qxQMfv6m1FdkxUQ Research paper on real time speech diarization 
 - http://multimedia.icsi.berkeley.edu/speaker-diarization/fast-speaker-diarization-using-python/ Implementation of real time speech diarization using a GPU and python 
 - http://stackoverflow.com/questions/20414667/cocktail-party-algorithm-svd-implementation-in-one-line-of-code
 
Hardware
 - https://www.amazon.co.uk/Logitech-C920-Pro-1080p-Webcam/dp/B006A2Q81M/ref=pd_lpo_147_bs_t_2/252-4253252-4860219?ie=UTF8&psc=1&refRID=32PJ73JZMFMDC2M2SY65
 - http://www.music-group.com/Categories/Behringer/Microphones/Condenser-Microphones/C-1/p/P0226
 - https://www.thomann.de/gb/behringer_c1_grossmembran_studiomikrofon.htm?glp=1&gclid=CjwKEAjwkJfABRDnhbPlx6WI4ncSJADMQqxdOF5GUPigkykS0pNrZld7GfeKUYxWyvJbDEhG_NK6OBoCduLw_wcB
 - http://www.logitech.com/en-gb/product/hd-pro-webcam-c920
 - http://www.ebay.co.uk/sch/Omni-directional-Camera-Microphone/83857/bn_70250/i.html
 - http://www.edmundoptics.com/cameras/

 - https://www.amazon.co.uk/PChero-Omnidirectional-Condenser-Microphone-Interviews/dp/B01GKSXTJE/ref=sr_1_sc_3?ie=UTF8&qid=1477403717&sr=8-3-spell&keywords=clip+on+usb+microphojne
 - https://www.amazon.co.uk/Tonor-Professional-Condenser-Microphone-Computer/dp/B01142EPO4/ref=sr_1_3?ie=UTF8&qid=1477404705&sr=8-3&keywords=usb+microphone
 - CHOSEN MIKES https://upload.wikimedia.org/wikipedia/commons/2/20/Mike_Tyson_Portrait_lighting_corrected.jpg

