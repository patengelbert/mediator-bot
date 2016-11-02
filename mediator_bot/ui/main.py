from flask import Flask, request, session, g, redirect, url_for, abort, \
     render_template, flash

class user_interface():

  def __init__(self):
    self.robot_mode="polite" #rude
    self.gui_mode="all" #none, etc varying levels of display reactions
    self.people=[]#("name", x,y) in relation to robot
    self.robot=["Robot", 0,0]
    self.mikes=[]#(x,y) we need to input defaults
    self.transcript=[] #("timestamp","name","words")] )list, 
    self.test_mode=0 #0- all, 1- etc level of involvement higher==less
    
    self.actions = {
      "too_loud" : self.quiet_down(),
      "speaking_over" : self.talk_over(),
      "dominated": self.talk_fair_amounts()                   
    }
 
}
  def __quiet_down(self):
    if(self.robot_mode!="polite"):
      #display message please speak quieter 
  
  def __talk_over(self):
    if(self.robot_mode!="polite"):
      #display message please speak one at a time
  
  def __talk_fair_amounts(self):
    if(self.robot_mode!="polite"):
      #display message "try not to dominate the conversation, amounts people are speaking"     
  
  def get_people(self):
    return self.people
    
  def set_people(self, people):
    self.people=people
    
  def get_robot_mode(self):
    return self.robot_mode
    
  def displayAction(self, action_type):
    #switch based on action to call sub function, 
    #e.g. its "too loud" so display "Talk softly please" 
    self.actions[action_type]    
    return 0
    
  #transcript receive is ["name", "timestamp", "words"]
  def receiveTranscript(self, transcript)
    #add to correct place in transcript list
    
app = Flask(__name__)
app.jinja_env.add_extension('jinja2.ext.loopcontrols')
    
@app.route('/')
def main_page():
  return render_template('/main.html')

if __name__ == "__main__":
  app.run(debug=True) 

