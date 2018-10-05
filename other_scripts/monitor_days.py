#!/usr/bin/env python
import uuid
import json
import copy
from collections import Counter
import numpy as np
import rospy

from std_msgs.msg import String, Int32MultiArray
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, \
  platform_control, core_state, core_control, core_config, bridge_config, bridge_stream
from miro_constants import miro
from miro_background.msg import Face, Faces, ActionUnit

#### just needed for a human readable format of the log file
class NoIndent(object):
    def __init__(self, value):
        self.value = value

class NoIndentEncoder(json.JSONEncoder):
    def __init__(self, *args, **kwargs):
        super(NoIndentEncoder, self).__init__(*args, **kwargs)
        self.kwargs = dict(kwargs)
        del self.kwargs['indent']
        self._replacement_map = {}

    def default(self, o):
        if isinstance(o, NoIndent):
            key = uuid.uuid4().hex
            self._replacement_map[key] = json.dumps(o.value, **self.kwargs)
            return "@@%s@@" % (key,)
        else:
            return super(NoIndentEncoder, self).default(o)

    def encode(self, o):
        result = super(NoIndentEncoder, self).encode(o)
        for k, v in self._replacement_map.iteritems():
            result = result.replace('"@@%s@@"' % (k,), v)
        return result

def change_indent(o):
  # if it is a dict, look at the keys
  if isinstance(o, dict):
    for k in o:
      # if the key is "events", don't indent the sublevel of the dict
      if k == "events":
        for x in o[k]:
          o[k][x] = NoIndent(o[k][x])
      # if the key is one of these, don't indent
      elif k in ["day", "day_group", "hour", "predicted_pattern", "observed_pattern"]:
        o[k] = NoIndent(o[k])
      # else look at the value
      else:
        change_indent(o[k])
  # if it is a list, look at all entries individually
  elif isinstance(o, list):
    [change_indent(d) for d in o]
  # or do nothing
  else:
    pass
####


class monitor:
  """Class to use MiRo to monitor how people interact with him."""
  def __init__(self):
    """Prepares MiRo

    E.g. setting all neccessary starting conditions and read the old log file
    """
    # First register the node
    rospy.init_node("miro_ros_client_py", anonymous=True)
    
    # Get all parameters from the launch file
    self.robot_name = rospy.get_param("~robot_name")
    self.demo_mode = rospy.get_param("~demo_mode")
    self.start_wait = rospy.get_param("~start_wait")
    self.interval_time = rospy.get_param("~interval_time")
    self.days = rospy.get_param("~days")
    self.hours_per_day = rospy.get_param("~intervals_per_day")
    self.monitor_file = rospy.get_param("~monitor_file") + "_" + str(self.days) + "_" + str(self.hours_per_day) + "_" + str(self.interval_time) + ".log"

    # Prepare the dictionaries for the log file
    self.event_template = {"happiness": [], "sadness": [], "blink": [], "surprise": [],
     "fear": [], "anger": [], "disgust": [], "contempt": [], "head_touch": [], "body_touch": [], "face_on_floor": []}
    self.hour_template = {"hour": [], "predicted_pattern": [], "observed_pattern": [], "events": copy.deepcopy(self.event_template)}
    self.hours_template = [copy.deepcopy(self.hour_template) for _ in range(self.hours_per_day)]
    self.day_template = {"day": [], "day_group": [], "hours": self.hours_template}

    # Start at day 0 and hour 0
    self.current_day = 0
    self.current_hour = 0
    # The Program is not yet at the end
    self.end = False
    # The Program is not active yet
    self.active = False

    # Add a shutdown hook, to savely close the program
    rospy.on_shutdown(self.shutdow_hook)

    # try to load the file 
    try:
      with open(self.monitor_file, "r") as filehandle:
        print("load the file")
        print(self.monitor_file)
        self.monitor_list = json.load(filehandle)
        # Increment the number of runs by one 
        self.col_id = self.monitor_list[0] + 1
        self.monitor_list[0] = self.col_id
    # if the file does not exist, create a new file
    except IOError:
      print("create a new file")
      self.col_id = 0
      self.monitor_list = [self.col_id, [copy.deepcopy(self.day_template) for _ in range(self.days)]]

    # Set all counters to 0
    self.happiness = 0
    self.sadness = 0
    self.blink = 0
    self.surprise = 0
    self.fear = 0
    self.anger = 0
    self.disgust = 0
    self.contempt = 0
    self.face_on_floor = 0 
    self.head_touch = 0
    self.body_touch = 0

    # Prepare miro's topic
    topic_root = "/miro/" + self.robot_name

    # Set up all publishers, we only use core/control and core/config
    self.pub_core_control = rospy.Publisher(topic_root + "/core/control", core_control, queue_size=0)
    self.pub_core_config = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)

    # Wait the time specified in launch file. This is necessary, because the publishers may not be ready immediately 
    rospy.sleep(self.start_wait)
    # After the waiting time, set the starting time
    self.start_time = rospy.Time.now().to_sec()

    # Set the dict to map from patterns to states
    self.pattern_to_state_dict = {"nothing": "Sleep", "face_on_floor": "Angry_disturbed", "happy_face": "Happy_active",
     "body_touch": "Happy_active", "head_touch": "Happy_calm", "sad_face": "Angry_calm", "neutral": "Neutral"}
    # Set the dict to map from states to /core/control messages 
    self.state_dict = {"Neutral": [0.5, 0.5, 0.5, 0.5, 1.0, 0.0], "Angry_disturbed": [0.0, 1.0, 0.0, 1.0, 1.0, 0.0], 
    "Angry_calm": [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], "Happy_active": [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
    "Happy_calm": [1.0, 0.0, 1.0, 0.0, 1.0, 0.0], "Sleep": [0.5, 0.0, 0.5, 0.0, 0.0, 1.0]}
    
    # Predict the pattern for the first time interval
    self.predict_next_pattern()

    # If the flag to use miro's demo behavior was set, activate the demo behavior
    if self.demo_mode == True:

      c = core_config()
      c.P2B_W_signals = c.P2B_W_signals | miro.MIRO_P2B_W_BRANCH_ENABLE
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ADJUST_RTC ##
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_VALENCE_DYNAMICS
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_AROUSAL_DYNAMICS
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE_SLEEP ##
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_CLOCK ##
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_WAKEFULNESS ##
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_TOUCH
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_LIGHT
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_SOUND
      c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_ACCEL
      # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_SLEEP_BLOCKED
      # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_RANDOMIZE_VALENCE
      # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FAST_SLEEP_DYNAMICS
      c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_ENABLE
      c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_LIGHT
      c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_TAIL
      c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_EYELIDS
      # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_EARS ## ?
      c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_VOCAL
      c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_BODY
      # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_DEBUG_SONAR
      # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_NO_PIRATE_NOISES
      # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_DO_PIRATE_NOISES
      c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_ENABLE
      # c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_DEBUG
      # c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_FORCE_MULL
      # c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_RANDOMIZE_ORIENT
      # c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_DISABLE_HALT
      c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_MODULATE_BY_SONAR
      c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_ENABLE
      # c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_RESET_KC_INTEGRATORS
      # c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH ## no movement
      # c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_MOTION
      # c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_TRANSLATION
      c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_INTO_SONAR
      # c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_ENABLE_POS_CONTROL
      c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_ENABLE_CLIFF_REFLEX
      c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_ENABLE
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_IGNORE_AUDIO
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_IGNORE_VIDEO
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SEND_PRIORITY
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SEND_OTHER
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_NO_REAFF_COMPROMISE
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_NO_SUPPRESS
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SHOW_COMPROMISE
      # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SHOW_TEST_PATTERN
      # c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_TEST_ALARM
      # c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_NO_I2C_BUSY_ALARM
      c.msg_flags = c.FLAG_UPDATE_SIGNALS

      # Publish the core/control message to activate miro's core behavior
      self.pub_core_config.publish(c)  
   
    # Add all subscribers
    # Platform state
    self.platform_state = rospy.Subscriber(topic_root + "/platform/state", platform_state, self.callback_state, queue_size=1)
    # Face on the florr
    self.face_sub = rospy.Subscriber("/faceCoord", Int32MultiArray, self.callback_faces)
    # Face emotions
    self.face_sub = rospy.Subscriber("/faces", Faces, self.callback_emotions)
    # Platform sensors (used for Touch)
    self.sub_platform_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_sensors)


  def publish_state(self):
    """Publishes the core control message that matches to Miro's current state"""
    # Get the information for the current state using the current predicted pattern
    state_array = self.state_dict[self.pattern_to_state_dict[self.current_predicted_pattern]]
    # Construct the core control message
    q = core_control()
    q.mood_drive_target.valence = state_array[0]
    q.mood_drive_target.arousal = state_array[1]
    q.mood_drive_gamma = self.drive
    q.emotion_drive_target.valence = state_array[2]
    q.emotion_drive_target.arousal = state_array[3]
    q.emotion_drive_gamma = self.drive
    q.sleep_drive_target.wakefulness = state_array[4]
    q.sleep_drive_target.pressure = state_array[5]
    q.sleep_drive_gamma = self.drive
    # Publish the message
    self.pub_core_control.publish(q)

    # Change the drive for the next message (we only want to set the state at the start of a time interval)
    self.drive = max(0, self.drive-0.1)

  def callback_emotions(self, faces):
    """Listens to the basic facial actions and put them together as emotions"""
    # Do nothing, if the monitor is not active yet
    if not self.active:
      return 

    # If at least one face is detected, look at the first face
    if len(faces.faces) != 0:

      # Prepare the variables for the different ActionUnits
      au1 = au2 = au4 = au5 = au6 = au7 = au9 = au12 = au14 = au15 = au20 = au23 = au26 = au45 = False 

      face_actions = faces.faces[0].action_units

      # Iterate over all ActionUnits
      for au in face_actions:

        # facial action gross behavior code 45: blink
        if au.name == "AU45" and au.presence == 1.0:
          print("blink detected" )
          au45 = True
          # Increment the blink counter
          self.blink += 1

        # inner brow raiser 
        if au.name == "AU01" and au.intensity > 2.0:
          print("inner brow raiser detected")
          au1 = True

        # outer brow raiser 
        if au.name == "AU02" and au.intensity > 2.0:
          print("outer brow raiser detected")
          au2 = True

        # brow lowerer 
        if au.name == "AU04" and au.presence == 1.0:
          print("brow lowerer detected")
          au4 = True

        # upper lid raiser
        if au.name == "AU05" and au.intensity > 2.0:
          print("upper lid raiser detected")
          au5 = True

        # cheeck raiser
        if au.name == "AU06" and au.presence == 1.0:
          print("cheek raiser detected")
          au6 = True

        # lid tightener
        if au.name == "AU07" and au.presence == 1.0:
          print("lid tightener detected")
          au7 = True

         # nose wrinkler 
        if au.name == "AU09" and au.presence == 1.0:
          print("nose wrinkler detected")
          au9 = True
      
        # lip corner puller 
        if au.name == "AU12" and au.presence == 1.0:
          print("lip corner puller detected")
          au12 = True

        # dimpler
        if au.name == "AU14" and au.presence == 1.0:
          print("dimpler detected")
          au14 = True
                  
        # lip corner depressor
        if au.name == "AU15" and au.intensity > 2.0:
          print("lip corner depressor detected")
          au15 = True

        # lip stretcher 
        if au.name == "AU20" and au.presence == 1.0:
          print("lip stretcher detected")
          au20 = True

        # lip tigthener 
        if au.name == "AU23" and au.presence == 1.0:
          print("lip tightener detected")
          au23 = True

        # jaw drop 
        if au.name == "AU26" and au.intensity > 2.0:
          print("jaw drop detected")
          au26 = True

        # place for other action units

      # Detect emotions (combinations of facial action units)

      # Happiness 6 + 12
      if au6 is True and au12 is True:
        print("happiness detected")
        self.happiness += 1

      # Sadness 1 + 4 + 15
      if au1 is True and au4 is True and au15 is True:
        print("sadness detected")
        self.sadness += 1
               
      # Surprise 1 + 2 + 5 + 26 
      if au1 is True and au2 is True and au5 is True and au26 is True:
        print("surprise detected")
        self.surprise += 1
      
      # Fear 1 + 2 + 4 + 5 + 7 + 20 + 26
      if au1 is True and au2 is True and au4 is True and au5 is True and au7 is True and au20 is True and au26 is True:
        print("fear detected")
        self.fear += 1

      # Anger 4 + 5 + 7 + 23
      if au4 is True and au5 is True and au7 is True and au23 is True:
        print("anger detected")
        self.anger += 1

      # Disgust 9 + 15 (+16 not recongized, therefore ignored)
      if au9 is True and au15 is True:
        print("disgust detected")
        self.disgust += 1

      # Contempt 12 + 14 (on one side of the face only, buy this implementation can't distinguish this, therefore more like another happiness)
      if au12 is True and au14 is True:
        print("contempt detected")
        self.contempt += 1

  def callback_faces(self, face_array):
    """Listen to the face on the floor message"""
    # Do nothing if the monitor is not yet active
    if not self.active:
      return

    # Increment the counter, if a face was detected 
    if face_array.data[1] != 0:
        self.face_on_floor += 1


  def predict_next_pattern(self):
    """Predict the pattern for the next time interval using the log file."""
    # Get the dict for the current day
    self.current_day_dict = self.monitor_list[1][self.current_day]
    
    # First hour only, log the day
    if self.current_hour == 0:
      self.current_day_dict["day"].append(self.current_day)
    # Last hour only, log the day-group
    if self.current_hour == self.hours_per_day - 1:
      self.current_day_dict["day_group"].append("Unknown")

    # Get the dict for the current hour
    self.current_hour_dict = self.current_day_dict["hours"][self.current_hour]
    self.current_hour_dict["hour"].append(self.current_hour)

    # Predict the pattern for the next period
    # First get the two most common observed patterns for that time interval in the past
    old_patterns = Counter(self.current_hour_dict["observed_pattern"]).most_common(2)

    # If no knowledge or knowledge is ambiguous, predict nothing 
    if len(old_patterns) == 0 or (len(old_patterns) == 2 and old_patterns[0][1] == old_patterns[1][1]):
      self.current_predicted_pattern = "neutral"
    # Else simply choose the pattern that occured most often
    # Could do some other stuff, e.g. rating last observed_pattern for that interval higher 
    # or take the previous observed_pattern into account, etc. 
    else:
      self.current_predicted_pattern = old_patterns[0][0]

    # Log the predicted pattern
    self.current_hour_dict["predicted_pattern"].append(self.current_predicted_pattern)

    # We want to set miro's internal state according to the predicted pattern
    self.drive = 1.0

  def callback_state(self, state):
    """Listens to the platform state message."""
    # If the program is not active yet, activate it 
    if not self.active:
      self.active = True

    else:
      # Check if the next interval has started 
      if rospy.Time.now().to_sec() > self.start_time + (self.current_day * self.hours_per_day + self.current_hour + 1) * self.interval_time:
       
        # Which pattern was actually observed?
        # Was there a face on the floor? (highest priority)
        if self.face_on_floor > 0:
          observed_pattern = "face_on_floor"
        # Were happy or sad faces present? (middle priority)
        elif (self.happiness + self.sadness) > 0:
          if self.happiness > self.sadness:
            observed_pattern = "happy_face"
          else:
            observed_pattern = "sad_face"
        # Was miro touched? (lowest priorioty)
        elif (self.body_touch + self.head_touch) > 0:
          if self.body_touch > self.head_touch:
            observed_pattern = "body_touch"
          else:
            observed_pattern = "head_touch"
        # Else nothing happened
        else:
          observed_pattern = "nothing"
        # Log the observed pattern
        self.current_hour_dict["observed_pattern"].append(observed_pattern)

        # Check if observed and predicted pattern are the same
        if self.current_predicted_pattern == observed_pattern:
          # do something
          pass
        else:
          # do something
          pass
          
        # Get the corresponding event dict
        current_event_dict = self.current_hour_dict["events"]
        # Log all basic units
        current_event_dict["happiness"].append(self.happiness)
        current_event_dict["sadness"].append(self.sadness)
        current_event_dict["blink"].append(self.blink)
        current_event_dict["surprise"].append(self.surprise)
        current_event_dict["fear"].append(self.fear)
        current_event_dict["anger"].append(self.anger)
        current_event_dict["disgust"].append(self.disgust)
        current_event_dict["contempt"].append(self.contempt)
        current_event_dict["head_touch"].append(self.head_touch)
        current_event_dict["body_touch"].append(self.body_touch)        
        current_event_dict["face_on_floor"].append(self.face_on_floor)

        # Reset all counters
        self.happiness = 0
        self.sadness = 0
        self.blink = 0
        self.surprise = 0
        self.fear = 0
        self.anger = 0
        self.disgust = 0
        self.contempt = 0
        self.face_on_floor = 0 
        self.head_touch = 0
        self.body_touch = 0

        # Count up current hour (and eventually day)
        self.current_hour = (self.current_hour + 1) % self.hours_per_day
        if self.current_hour == 0:
          self.current_day = (self.current_day + 1) % self.days

        # If one week has passed, shutdown the program
        if self.current_day == 0 and self.current_hour == 0:
          self.end = True
          rospy.signal_shutdown("one week passed")
 
        # Predict pattern for the next period
        self.predict_next_pattern()       

  def callback_sensors(self, sensors):
    """Listen to the platform/sensor messages to detect touch."""
    # Increment head and body touch if touched
    self.head_touch += np.sum(np.array(bytearray(sensors.touch_head)).astype(np.float)) 
    self.body_touch += np.sum(np.array(bytearray(sensors.touch_body)).astype(np.float)) 


  def shutdow_hook(self):
    """Stops the core behavior and saves the file, when the program is stopped"""
    # Stop core behavior
    c = core_config()
    c.msg_flags = c.FLAG_UPDATE_SIGNALS
    self.pub_core_config.publish(c) 

    # Save only if a full week has passed 
    if self.end == True:
      # Try to save the file
      try:
        with open(self.monitor_file, "w") as filehandle:
          # Change the indentation for a nice humanreadable format
          change_indent(self.monitor_list)
          # Save the file
          filehandle.write(json.dumps(self.monitor_list, cls=NoIndentEncoder, indent=4))
          print("file saved")
      except IOError:
        print("error at saving"     )

  def main(self):
    """Main loop publishes the core/control message 10 times per second"""
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
      rate.sleep()
      # Publish the core/control message
      self.publish_state()
     
if __name__ == '__main__':
  # Create and run the monitor class
  monitor = monitor()
  monitor.main()
