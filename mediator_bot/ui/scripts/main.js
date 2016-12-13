var POLITE=0;
var RUDE=1;
var FACE=0;
var MESSAGE=1;
var BAR=2;
var PIE=3;
var SET_UP=4;
var TEST_MODE=2;
var repeat_sentence="<b>Speak the following sentence:</b> Donald Trump lost the popular vote, in a true democratic system he would not be president ";
var registration_mode=0;

function user_interface(){
  var main=this;
  this.robot_mode=POLITE; //rude or polite
  this.gui_mode="all"; //varying level of display reaction to inputs
  this.test_mode=2; //0- face, 1- sentences, 2- bargraph, 3- pie
  this.people=[]; //{person:"samamtha", percent: 30,x:5,y:5},{person:"richard", percent: 70,x:5,y:5}
  this.robot=["Robot",0,0];
  this.mikes=[(20,30), (10,10)];   
  this.transcript=[];
  this.display_lifespan=5;
  this.current_face="happy";
  this.current_action="too_loud"; 
  this.current_person="Bob";
  this.message="";
  
  this.actions={none: ["", ""], 
                speaking_over: ["Please don't speak over eachother", "don't speak over, let everyone get a chance"],
                too_loud: ["Please speak more softly"," don't speak so loud"] ,
                dominated: ["Please let everyone get an even say"," don't dominate the conversation"],
		too_quiet: ["Please speak a little louder", " come on speak up!"]
  };
  this.demo = function (){
    this.robot_mode=POLITE;
    this.gui_mode="all";
    this.test_mode=0;
    this.people=[("samamtha", 0,5,5),("richard", 0,5,5)]; //person, percentage spoken, x, y
    this.robot=["Robot",0,0];
    this.mikes=[];   
    this.transcript=[("1","richard","hello"),
                    ("2","samamtha","hello"),
                    ("3","richard","i just said that")];

    this.display_lifespan=5;
    this.current_face="happy";
    this.current_action="too_loud"; 
    this.message="";
  };
 /*
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
        */
};
var bar_colors= new Array("#22B2A8", "#FF9864", "#FFA97E", "#B29282");
var ui=new user_interface();

//When receive percentages, set ui vars, then call this if its on display
var create_bargraph=function(){
  //clears previous
  $("#bar_graph").empty();
  $("#bar_graph").append("<h2>Speech Quantity Analysis</h2>");
  
  //adds bars for each person to graph
  for(var person=0; person<ui.people.length;person++){
    $("#bar_graph").append('<div id="bar"></div>');
  }
  
  /*
  
      var newdiv = document.createElement("div");
    //div.style.width = String(ui.people[person][1])+"%";
    //div.innerHTML = ui.people[person][0];
    newdiv.width("10%");
    newdiv.height("10%");
    newdiv.css('background','#8ec252');
    $("#bar_graph").append(newdiv);
    */
  
  //controls color of bar graph
  $("#bar_graph").find("div").each(function(index) {
        var bar=index%bar_colors.length;
        $(this).css( "background-color", bar_colors[bar] );
        $(this).text(function(){
          return ui.people[index].person;
        });
        //innerHTML=ui.people[index][0];
        $(this).css( "width", ui.people[index].percent + "%");
  });
  
};

//action topic is: action_type, mood, person involved
var set_action=function(action, mood, person){
  ui.current_action=action.toLowerCase();
  ui.current_person=person;
  //ui.current_face=mood;
  if(ui.robot_mode==POLITE){
    ui.message=ui.actions[ui.current_action][ui.robot_mode];
  //rude mode, more direct
  }else{
    ui.message=person+ui.actions[action][ui.robot_mode];
  }
  //switch face displayed according to mood
  set_face_mode(mood);
  set_message(ui.message);
};

var set_message=function(message){
  $("#small_msg").text(function(){
    return message;
  });
  $("#big_msg").text(function(){
    return message;
  });
  
};

var set_test_mode=function(test_mode){
  var test_array=["#container","#big_msg", "#bar_graph", "#pie_graph", "#set_up"];
   if(test_mode==SET_UP && ui.test_mode!=SET_UP){TEST_MODE=ui.test_mode;}
   //if(ui.test_mode==SET_UP && test_mode!=SET_UP
    var current_test_mode=test_array[ui.test_mode];
    $(current_test_mode).css( "visibility", "hidden");
  
 
    var new_test_mode=test_array[test_mode];
    $(new_test_mode).css( "visibility", "visible" );
  
  
  switch(test_mode){
    //face
    case 0:
      set_action(ui.current_action,ui.current_face, ui.current_person);
      break;
    //sentences
    case 1:
      set_action(ui.current_action,ui.current_face, ui.current_person);
      break;
    //bar
    case 2:
      create_bargraph();
      break;
    case 3:
      //$("#pie_graph").css( "visibility", visible );
      break;
    case 4:
      
      break;
    default:
      break;
  }
  ui.test_mode=test_mode;
};

var set_face_mode=function(face){
  //var currentface="#face_"+ui.current_face;
  var new_face="assets/"+face+"_face.png";
  $("#face img").attr("src", new_face);
 // $(currentface).css( "visibility", "hidden" );
  //var newface="#face_"+face;
  //$(currentface).css( "visibility", "visible" );
/*  
  switch(face){
    //neutral
    case "neutral":
      $("#face_netural").css( "visibility", visible );
      break;
    //happy
    case "happy":
      $("#face_happy").css( "visibility", visible );
      break;
    //sad
    case "sad":
      $("#face_sad").css( "visibility", visible );
      break;
    //angry
    case "angry":
      $("#face_angry").css( "visibility", visible );
      break;
    //shocked
    case "shocked":
      $("#face_shocked").css( "visibility", visible );
      break;
    default:
      break;
  }
  */
  ui.current_face=face;
};






$(document).ready(function(){
  //demo
  //$("#set_up").css("visibility", "hidden");
  set_test_mode(BAR); 
  //set_action("too_loud","happy", "Bob");
$("#set_up_link").click(function(){
  //alert("sup");
  
  set_test_mode(SET_UP);
});
$("#home_link").click(function(){

  var requestRecog=new ROSLIB.ServiceRequest({
  });

  startRecognition.callService(requestRecog, function(result){
    set_test_mode(TEST_MODE);
    $('#new_people').innerHTML="";
  });    

  //set_test_mode(TEST_MODE);
});


$('#change_mode').click(function(){
  var newMode=$('#test_modes').val();
  if(ui.test_mode==SET_UP){
    TEST_MODE=newMode;
  }else{
    set_test_mode(newMode);
  }
});
/*
$("#submit_name").click(function(){
  console.log("sup");
  var inputs=$('#person_setup :input').val();
  var register_person=new ROSLIB.Message({
    Person: inputs,
    Success:true
  });
  person_setup.publish(register_person);
  $('#person_setup :input').val('');
});
*/
var inputs;
$('#submit_name').click(function(){
  
  if(registration_mode==0){
    if($('#person_setup :input').val()==""){
      $("#sentence_to_speak").html("You need to enter a name");
    }else{
      inputs=$('#person_setup :input').val();
      $('#person_setup :input').val("");
      var requestStart=new ROSLIB.ServiceRequest({
        name:inputs
      });
      registerPerson.callService(requestStart, function(result){
        if(result.success){
          $('#submit_name').text("Stop Person Registration");
          console.log("name successfully submitted");
          $("#sentence_to_speak").html(repeat_sentence);
          registration_mode=1;
        }

      });
    }    
  }else if(registration_mode==1){
    var requestStop=new ROSLIB.ServiceRequest({
    });
    endEnrollment.callService(requestStop,function(result){
     if(result.success){
        $('#submit_name').text("Start Person Registration");
        ui.people.push({person:inputs,percent: 0,x:0,y:0});
        $('#new_people').append(inputs+ "<br>");
        registration_mode=0;
        $("#sentence_to_speak").text("");
      }
    });
  }




  });

});


//Connecting to ROS
//------------------------------------------------
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});
ros.on('connection', function() {
  console.log('Connected to websocket server.');
});
   
ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});
   
ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});
//Service
var registerPerson= new ROSLIB.Service({
  ros : ros,
  name : '/StartEnrollment',
  serviceType: 'ui/StartEnrollment'
});
var startRecognition= new ROSLIB.Service({
  ros: ros,
  name: '/StartRecognition',
  messageType:'ui/StartRecognition'
});
var endEnrollment= new ROSLIB.Service({
  ros : ros,
  name: '/EndEnrollment',
  serviceType: 'ui/EndEnrollment'
});

//Subscribing to a topic
//----------
var action_listener= new ROSLIB.Topic({
  ros:ros,
  name:'/Action',
  messageType:'ui/Action'
});

var percent_listener=new ROSLIB.Topic({
  ros:ros,
  name:'/Percentage',
  messageType:'ui/Percentage'
});
var register_listener=new ROSLIB.Topic({
  ros:ros,
  name:'/Registration_complete',
  messageType:'ui/Registration_Complete'
});
var transcript_listener=new ROSLIB.Topic({
  ros:ros,
  name:'/Transcript',
  messageType:'ui/Transcript'
});

action_listener.subscribe(function(message){
  //alert("hello");
  console.log(message.Person + message.Action + message.Mood);
  //alert("hello world");
  set_action(message.Action, message.Mood.toLowerCase(), message.Person);
//  action_listener.unsubscribe();
});
percent_listener.subscribe(function(message){
  console.log(message.Person + message.Percentage);
  for(var i=0; i<ui.people.length; i++){
    if(ui.people[i]["person"]==message.Person){
      ui.people[i]["percent"]=message.Percentage;
      create_bargraph();
      //break;  
    }
  }
});
register_listener.subscribe(function(message){
  console.log(message.label + message.success);
  if(message.success){
    var index=ui.people.length;
    ui.people[index]={person: message.label,percent:  0, x:0, y:0};
  }
});
//Publishing a topic
//------------------------------------------
var person_setup=new ROSLIB.Topic({
  ros: ros,
  name: '/Register_Person',
  messageType: 'ui/Person_Register'
});

/*
var register_person= new ROSLIB.Message({
  Person: "Sam",
  Success: true
});
*/
/*
$("#submit_name").click(function(){
  console.log("sup");
  var inputs=$('#person_setup :input').val();
  var register_person=new ROSLIB.Message({
    Person: inputs,
    Success:true
  });
  person_setup.publish(register_person);
  $('#person_setup :input').val('');

});
*/
/*
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
*/

/*

# Commented out as app was missing
@app.route('/')
def main_page():
  speech = [ (x, round(float(y)/page.total_speech*100)) for x,y in page.speech]
  return render_template('/main.html', message=page.actions[page.current_action], display_type=page.test_mode, people=page.people, transcript=page.transcript, speech=speech, face=page.current_face)
  
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
*/

