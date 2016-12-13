#!/usr/bin/env python
"""
-Feedback page forms/questions?
-JS auto update live feed
-ROS implementation
-python settings
-write paper
"""


from flask import Flask, request, session, g, redirect, url_for, abort, \
     render_template, flash
from flask_assets import Environment, Bundle

import rospy
from std_msgs.msg import String

app = Flask(__name__)
app.jinja_env.add_extension('jinja2.ext.loopcontrols')

class user_interface():
    def __init__(self):
        self.robot_mode = "polite"  # rude
        self.gui_mode = "all"  # none, etc varying levels of display reactions
        self.people = []  # ("name", x,y) in relation to robot
        self.robot = ["Robot", 0, 0]
        self.mikes = []  # (x,y) we need to input defaults
        self.transcript = []  # ("timestamp","name","words")] )list,
        self.test_mode = 0  # 0- all, 1- etc level of involvement higher==less
        self.display_lifespan=5
        self.current_action="none"
        self.actions = {
            "too_loud": self.quiet_down(),
            "speaking_over": self.talk_over(),
            "dominated": self.talk_fair_amounts()
        }  
        self.message=""
    def demo(self):
        self.robot_mode = "polite"  # rude
        self.gui_mode = "all"  # none, etc varying levels of display reactions
        self.people = [("samamtha",5,5),("richard",5,5)]  # ("name", x,y) in relation to robot
        self.robot = ["Robot", 0, 0]
        self.mikes = [(20,30), (10,10)]  # (x,y) we need to input defaults
        self.transcript = [("1","richard","hello"),("2","samamtha","hello"),("3","richard","i just said that")]  # ("timestamp","name","words")] )list,
        self.speech=[("richard",21),("samamtha", 5)]#update every time transcript added or calculate before send with marker to check if recalculation needed
        self.total_speech=26
        self.test_mode = 0  # 0- all, 1- etc level of involvement higher==less
      
    def quiet_down(self):
        if (self.robot_mode != "polite"):
          message="Please do not speak so loudly"
        else:
          message="Pipe da fuq down"
        return message
            
    def talk_over(self):
        if (self.robot_mode != "polite"):
          message="Please don't speak over eachother"
        else:
          message="Speaking over each other leads to less productive conversations"
        return message

    def talk_fair_amounts(self):
        if (self.robot_mode != "polite"):
          message="Please try and let everyone's viewpoint be heard"
        else:
          message="Robot stop dominating the conversation"
        return message

    def get_people(self):
        return self.people

    def set_people(self, people):
        self.people = people

    def get_robot_mode(self):
        return self.robot_mode

    def displayAction(self, action_type):
        
        #if request.path.startswith('/live'):
        self.message=self.actions[action_type] 
        redirect(url_for('live_page'))
        time.sleep(10)
        self.message=""
        redirect(url_for('live_page'))
        #elif self.robot_mode=="rude":
           #potential to force link to live page
           
          
        # switch based on action to call sub function,
        # e.g. its "too loud" so display "Talk softly please"
               
        return 0
        
    def receiveTranscript(self, transcript):
        # add to correct place in transcript list
        return 0
    


# Commented out as app was missing
@app.route('/')
def main_page():
  return render_template('/home.html')
  
@app.route('/live')
def live_page():
  return render_template('/live.html', message=page.message)
  
@app.route('/analysis')
def stats_page():
  #speech= map(lambda x, y: )
  #          page.speech[j],round((x / page.total_speech)*100) for j in page.speech[:][0] for x in page.speech[:][1]
  speech = [ (x, round(float(y)/page.total_speech*100)) for x,y in page.speech]
  
 
  return render_template('/analysis.html', people=page.people, transcript=page.transcript, speech=speech)
  
@app.route('/feedback')
def feedback_page():
  return render_template('/feedback.html')

@app.route('/set_up')
def setup_page():
  return render_template('/set_up.html')
  
@app.after_request
def add_header(response):
    """
    Add headers to both force latest IE rendering engine or Chrome Frame,
    and also to cache the rendered page for 10 minutes.
    """
    response.headers['X-UA-Compatible'] = 'IE=Edge,chrome=1'
    response.headers['Cache-Control'] = 'public, max-age=0'
    return response

app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 10

#assets = Environment(app)
"""
sass = Bundle('*.sass' filters='sass', output='gen/sass.css')
all_css = Bundle('styles/interaction.css', sass,
                 filters='cssmin', output="gen/all.css")
"""
"""
css = Bundle('styles/interaction.css', 
             filters='less,cssmin', output='gen/packed.css')

assets.register('css_all', css)

"""

"""
css = Bundle('styles/interaction.css',
            filters='jsmin', output='gen/packed.js')
css.extra='styles/interaction.css'
"""

#assets.register('css_all', all_css)


"""
@app.context_processor
def override_url_for():
    return dict(url_for=dated_url_for)

def dated_url_for(endpoint, **values):
    if endpoint == 'static':
        filename = values.get('filename', None)
        if filename:
            file_path = os.path.join(app.root_path,
                                     endpoint, filename)
            values['q'] = int(os.stat(file_path).st_mtime)
    return url_for(endpoint, **values)
"""
page=user_interface()
page.demo()

#Node stuff

def action_callback(data):
  page.=data.Action
  page.mood=data.Mood
  person_involved=data.Person

def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("Action", String, action_callback)
  rospy.spin()


if __name__ == "__main__":
  listener()
  app.run(debug=True)
  
