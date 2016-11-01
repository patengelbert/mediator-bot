from flask import Flask, request, session, g, redirect, url_for, abort, \
     render_template, flash

class user_interface():

  def __init__(self):
    self.robot_mode="polite" #rude
    self.gui_mode="all" #none, etc varying levels of display reactions
    self.people=[]#("name", x,y) in relation to robot
    self.robot=["Robot", 0,0]
    self.mikes=[]#(x,y) we need to input defaults
    
  def get_people(self):
    return self.people
    
  def set_people(self, people):
    self.people=people
    
  def get_robot_mode(self):
    return self.robot_mode
    
  def displayAction(self, action_type):
    #switch based on action to call sub function, 
    #e.g. its "too loud" so display "Talk softly please"  
    return 0
    
app = Flask(__name__)
app.jinja_env.add_extension('jinja2.ext.loopcontrols')
    
@app.route('/')
def main_page():
  return render_template('/main.html')

if __name__ == "__main__":
  app.run(debug=True) 

