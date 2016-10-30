# Mediator-Bot

[![Build Status](https://travis-ci.com/patengelbert/mediator-bot.svg?token=imo3fxmH6VGoPsvCNfup&branch=master)](https://travis-ci.com/patengelbert/mediator-bot)

The main function of the mediator-bot is to act as a mediator for a conversation/debate/meeting, trying to balance the conversation so that each participant is given the chance to speak.

Using:
* Speech Recognition - voice recognition, determining if multiple people are talking and split into separate channels, produce transcript, determine volume level and tone of conversation
* Face Recognition - match face with name and speech, additional detection of speaking
* Processing - determine the balance of the discussion, decide what actions by the robot are required, display visual representation of conversation statistics
* Robot - gestures and speech to direct conversation, interaction (e.g. turn head towards speaker)

Possible test - hold two separate meetings, one with a human mediator and one with the mediator-bot, gather participants' opinions on which was preferred.

## Requirements
- Python 2.7 (Project should be compatible with both it and Python 3.4)
- Make
- Python 2.7 virtualenv ```pip install virtualenv```
- Python 2.7 Sphinx ```pip install sphinx```

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

## System

![Alt text](https://github.com/patengelbert/mediator-bot/raw/master/images/System Overview.png?raw=true)

##Timeline

| Date          | Deadline          | Deliverables    |
| ------------- |:-----------------| :--------------|
| 3/11/16       | 4 page Design Report     | - [ ] Research Hypothesis with stats<br>  - [x] System Design Diagram<br>  - [ ] your proposed experimental validation methodology     |
| 17/11/16    | 4 page Individual Component Paper         |  - [ ]     Report the state of the subcomponent you are responsible for <br> - [ ] Design to be followed for the subcomponent      |
| 15/12/16 | Final Presentation, Live demo          |   - [ ] It Works         |
| 22/12/16 | 8 page Final Report Submission (LaTex)         |   - [ ] Final System Design<br> - [ ] Experimental results  analysis <br> - [ ] Compressed file with all experimental results & readme.md <br> - [ ] Bonus 5% video-Explain hypothesis, experimental findings, potential use cases <br>*Look at examples on BB*       |

## Resources

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
 - CHOSEN MIKES https://www.amazon.co.uk/Tonor-Professional-Condenser-Microphone-Computer/dp/B01142EPO4/ref=sr_1_3?ie=UTF8&qid=1477404705&sr=8-3&keywords=usb+microphone

