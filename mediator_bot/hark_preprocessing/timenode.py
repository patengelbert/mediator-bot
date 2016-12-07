#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from hark_msgs.msg import HarkSrcWave
from mediator_bot_msgs.msg import MedBotSpeechTiming

class TimeAllocator:
    ''' 
    Module assigning a weighting to each speaker to show their current priority
    in the conversation, relative to all the other participants.

    The participants are identified by the direction they are sitting from the 
    recording devices. This naive approach may be changed.
    '''
    def callback(self,data):
        '''        
        Callback function to show who is currently speaking
        '''
        # Reset all talking flags
        self.talking = [False] * len(self.talking)
        # Set to talking if theres a source in the speakers direction
        for src in data.src:
            self.talking[self.assign(src.azimuth)] = True   
    
    def assign(self, angle):    
        # Return the person position associated with the direction
        if angle >= -90.0 and angle <= -30.0:
            return 0 
        elif angle > -30.0 and angle <= 30.0:
            return 1
        elif angle > 30.0 and angle <= 90.0:
            return 2
        return -1

    def __init__(self, debugLevel=rospy.INFO):
        rospy.init_node('TimeAllocator', anonymous=True, log_level=debugLevel)

        rospy.loginfo("Initialising TimeAllocator node")

        # TODO set these parameters from the rosparam server
        # time allocation containers
        self.numSpeakers = 3
        self.speakerIDs = ["Speaker1","Speaker2","Speaker3"]
        self.weightings = [0.0] * self.numSpeakers
        self.talking = [False] * self.numSpeakers

        # parameters for time allocation. TODO agree on a standard range and scale
        self.decFactor = 0.5          # per second
        self.incFactor = 1            # per second
        self.max = 10                 # max total value
        self.warningThreshold = 5.0   # warning when talking too much
        self.warningMsg = "Shhh"

        # Flag. Module in init stage
        self.init=True

        # Timing
        self.rate = 10.0

        # Create subscribers/publishers
        rospy.Subscriber("HarkSrcWave", HarkSrcWave, self.callback)
        self.pub = rospy.Publisher('/speech_timing',MedBotSpeechTiming,queue_size=10,latch=True)

    def printMultProgress(self, iteration, total, prefix, suffix, decimals = 1, barLength = 50):
        """
        Debug function.
        Source: http://stackoverflow.com/a/34325723

        Call in a loop to create terminal progress bar
        @params:
            iteration   - Required  : current iteration (Int)
            total       - Required  : total iterations (Int)
            prefix      - Optional  : prefix string (Str)
            suffix      - Optional  : suffix string (Str)
            decimals    - Optional  : positive number of decimals in percent complete (Int)
            barLength   - Optional  : character length of bar (Int)
        """

        CURSOR_UP_ONE = '\x1b[1A'
        ERASE_LINE = '\x1b[2K'
        if not self.init:
            sys.stdout.write(CURSOR_UP_ONE + CURSOR_UP_ONE + CURSOR_UP_ONE)
        self.init=False

        for i,a in enumerate(iteration):
            formatStr = "{0:." + str(decimals) + "f}"
            percent = formatStr.format(100 * (iteration[i] / float(total[i])))
            filledLength = int(round(barLength * iteration[i] / float(total[i])))
            bar = unichr(0x2588)  * filledLength + '-' * (barLength - filledLength)
            sys.stdout.write('%s |%s| %s%s %s\n' % (prefix[i], bar, percent, '%', suffix[i])),

        if iteration == total:
            sys.stdout.write('\n')
        sys.stdout.flush()

    def calc(self):
        '''
        Main function for calculating the current time allocation for each
        speaker. The result is printed on the terminal. 
        '''
        # Increment/decrement the time allocation based on if they are speaking
        for i in range(1,self.numSpeakers):
            if self.talking[i] and self.weightings[i] < self.max:
                self.weightings[i] = self.weightings[i]+float(self.incFactor/self.rate)
            elif not self.talking[i] and self.weightings[i]>0:
                self.weightings[i] = self.weightings[i]-float(self.decFactor/self.rate)
    
        # Set the warning msg to print if the weighting is above the threshold
        suffix = [self.warningMsg if x > self.warningThreshold else "" for x in self.weightings]
        # Print out the current speach participation levels
        self.printMultProgress(self.weightings,[self.max,self.max,self.max], \
                                   self.speakerIDs,suffix)

    def packTimingMsg(self):
        msg = MedBotSpeechTiming()
        msg.header.stamp = rospy.Time.now()
        msg.num_speakers = self.numSpeakers
        msg.speaker_id = self.speakerIDs
        msg.weighting = self.weightings
        return msg

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.calc()
            msg = self.packTimingMsg()
            self.pub.publish(msg)
            r.sleep()
			  
if __name__ == '__main__':
    t = TimeAllocator()
    t.run()
