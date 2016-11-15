## Action Library

This library contains operations for controlling the movement of the NAO. These actions will be called by a keyword passed by the AI. There may be multiple actions for the same keyword, in this case the actions will be grouped together by keyword and when the library receives a call for that keyword it will choose one of the actions at random from that keyword group.
The action operations will include:
* When it is someone's turn to speak - the NAO points to the person who should speak
* When the volume exceeds the acceptable level - the NAO makes a "quiet" gesture (an up and down movement of the hand, repeated a number of times)
* When multiple people speak at the same time - a "stop" gesture (arm forwards, hand spread with fingers vertical) or a combination of the "quiet" gesture and pointing to the person that should speak
* Certain "keywords" are spoken, or certain emotions shown - related gesture (this will depend on the keywords and emotions that we identify as requiring attention)
* When nothing is happening (ie. "normal" coversation) - the NAO will continue to do small movements (eg. look at speaker, nod, appear to breathe etc.) at random until prompted by the AI to do something else

The action library will need to receive one keyword and optionally a direction for a single person (for pointing or singling out).


## Response Library

This library contains operations for controlling non-movement reactions of the NAO. This will primarily be speech but might also include lighting of the LED sections or any other non-movement actions that the NAO is capable of. As with the Action Library, the responses will be grouped by a keyword and will be chosen at random should the group contain more than one operation.
The actions of this library will include:
* When it is someone's turn to speak - say the person's name (eg. "John, what are your views?")
* When the volume exceeds the acceptable level - the NAO may not speak in this instance as it may not be heard, it may wait for the conversation to become quieter and then speak (eg. "that was a bit loud")
* When multiple people speak at the same time - as above wait until it is quiet then speak (eg. "one speaker at a time please", "could we hear what you have to say first John")
* Certain "keywords" are spoken, or certain emotions shown - related speech (this will depend on the keywords and emotions that we identify as requiring attention)

The response library will need to receive one keyword and optionally the name of a single person (for seperating single people to speak).


## Flow Diagram

![HCR_Libraries](https://github.com/patengelbert/mediator-bot/blob/master/images/HCR_Libraries.png?raw=true)

## Simplified System Diagram

![Simplified_HCR](https://github.com/patengelbert/mediator-bot/blob/jb-branch/images/Simplified_HCR.png?raw=true)

## Component Report

[Component_Report](https://www.overleaf.com/7034599jzfgqcnqbdwd)

