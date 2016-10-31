from flask import Flask, request, session, g, redirect, url_for, abort, \
     render_template, flash

app = Flask(__name__)
app.jinja_env.add_extension('jinja2.ext.loopcontrols')

class user_interface():
  def __init__(self):
    self.robot_mode="quiet"
    self.people=[]#("name", x,y) in relation to robot
    self.robot=["Robot", 0,0]
    self.mikes=[]#(x,y) we need to input defaults
  def get_people(self):
    return self.people
  def set_people(self, people):
    self.people=people
  def get_robot_mode(self):
    return self.robot_mode

if __name__ == "__main__":
  app.run(debug=True) 

