#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray
import json
import copy
import random
from miro_background.msg import Face, Faces, ActionUnit
import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream
from miro_constants import miro
import uuid
import numpy as np

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
        print("default")
        if isinstance(o, NoIndent):
            key = uuid.uuid4().hex
            self._replacement_map[key] = json.dumps(o.value, **self.kwargs)
            return "@@%s@@" % (key,)
        else:
            return super(NoIndentEncoder, self).default(o)

    def encode(self, o):
        print("encode")
        result = super(NoIndentEncoder, self).encode(o)
        for k, v in self._replacement_map.iteritems():
            result = result.replace('"@@%s@@"' % (k,), v)
        return result


class monitor:
  def __init__(self):
    rospy.init_node("miro_ros_client_py", anonymous=True)
    self.active = False
    self.robot_name = rospy.get_param("~robot_name")
    self.demo_mode = rospy.get_param("~demo_mode")
    self.id = 0
    self.start_wait = rospy.get_param("~start_wait")
    self.interval_time = rospy.get_param("~interval_time")
    self.monitor_file = rospy.get_param("~monitor_file") + str(self.interval_time) + ".log"
    topic_root = "/miro/" + self.robot_name
    self.dict_template = {"col_id": [], "timeStamp": [], "happiness": [], "sadness": [], "blink": [], "surprise": [],
     "fear": [], "anger": [], "disgust": [], "contempt": [], "head_touch": [], "body_touch": [], "face_on_floor": []}
    self.start_time = rospy.Time.now().to_sec()

    rospy.on_shutdown(self.shutdow_hook)

    # try to load the file 
    try:
      with open(self.monitor_file, "r") as filehandle:
        print("load the file")
        print(self.monitor_file)
        # print(filehandle)
        self.monitor_list = json.load(filehandle)
        # print(self.monitor_list)
        self.col_id = self.monitor_list[0] + 1
        self.monitor_list[0] = self.col_id
    except IOError:
      print("create a new file")
      self.col_id = 0
      self.monitor_list = [self.col_id, copy.deepcopy(self.dict_template)]


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


    self.pub_core_control = rospy.Publisher(topic_root + "/core/control", core_control, queue_size=0)
    self.pub_core_config = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)

    # wait a specific amount of time
    rospy.sleep(self.start_wait)

    # send specific core_control message to start MiRo in a neutral state
    q = core_control()
    q.mood_drive_target.valence = 0.5
    q.mood_drive_target.arousal = 0.5
    q.mood_drive_gamma = 1
    q.emotion_drive_target.valence = 0.5
    q.emotion_drive_target.arousal = 0.5
    q.emotion_drive_gamma = 1 
    self.pub_core_control.publish(q)

    # prepare general core_control message
    self.core_message = core_control()
    # self.core_message.msg_flags = core_control.FLAG_SYNC_PLATFORM | core_control.FLAG_SYNC_CORE 
    self.core_message.emotion_drive_target.valence = 0.5
    self.core_message.emotion_drive_target.arousal = 0.5
    self.core_message.emotion_drive_gamma = 0.5

    # if flag was set activate demo mode
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
      self.pub_core_config.publish(c)  

      # listen to the core to adjust the state
      self.sub_core_state = rospy.Subscriber(topic_root + "/core/state", core_state, self.callback_core_state)
   

    # subscribe to the platform state topic to get the time
    self.platform_state = rospy.Subscriber(topic_root + "/platform/state", platform_state, self.callback_state, queue_size=1)

    # other subscribers 
    # face lying down
    self.face_sub = rospy.Subscriber("/faceCoord", Int32MultiArray, self.callback_faces)

    # face emotions
    self.face_sub = rospy.Subscriber("/faces", Faces, self.callback_emotions)

    # touch 
    self.sub_platform_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_sensors)

  def callback_emotions(self, faces):
    if not self.active:
      return 

    if abs(self.core_message.emotion_drive_target.valence - self.emotion_valence) < 0.1:
      self.core_message.emotion_drive_target.valence = self.emotion_valence
    else:
      print("oldmessage: ", self.core_message.emotion_drive_target.valence)
      print("new emotion: ", self.emotion_valence)

    if len(faces.faces) != 0:

      # prepare aus
      au1 = au2 = au4 = au5 = au6 = au7 = au9 = au12 = au14 = au15 = au20 = au23 = au26 = au45 = False 

      face_actions = faces.faces[0].action_units

      for au in face_actions:

        # print(au.name)
        # print(au.presence)

        # facial action gross behavior code 45: blink
        if au.name == "AU45" and au.presence == 1.0:
          print("blink detected" )
          au45 = True
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

      # detect emotions (combinations of facial action units)

      # Happiness 6 + 12
      if au6 is True and au12 is True:
        print("happiness detected")
        self.happiness += 1

        self.core_message.emotion_drive_target.valence = min(self.core_message.emotion_drive_target.valence + 0.05, 1)

      # Sadness 1 + 4 + 15
      if au1 is True and au4 is True and au15 is True:
        print("sadness detected")
        self.sadness += 1
        
        self.core_message.emotion_drive_target.valence = max(self.core_message.emotion_drive_target.valence - 0.05, 0)
       
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

      # Contempt 12 + 14
      if au12 is True and au14 is True:
        print("contempt detected")
        self.contempt += 1

  def callback_faces(self, face_array):
    if not self.active:
      return

    # face detected!
    if face_array.data[1] != 0:
        self.face_on_floor += 1


  def callback_state(self, state):
    if not self.active:
      self.active = True

    else:
      # check the time (and write to list)
      if rospy.Time.now().to_sec() > self.start_time + (self.id + 1) * self.interval_time:
        # write 
        try:
          current_dict = self.monitor_list[self.id + 1]
        except IndexError:
          current_dict = copy.deepcopy(self.dict_template)
          self.monitor_list.append(current_dict)

        current_dict["col_id"].append(self.col_id)
        current_dict["timeStamp"].append(rospy.Time.now().to_sec())
        current_dict["happiness"].append(self.happiness)
        current_dict["sadness"].append(self.sadness)
        current_dict["blink"].append(self.blink)
        current_dict["surprise"].append(self.surprise)
        current_dict["fear"].append(self.fear)
        current_dict["anger"].append(self.anger)
        current_dict["disgust"].append(self.disgust)
        current_dict["contempt"].append(self.contempt)
        current_dict["head_touch"].append(self.head_touch)
        current_dict["body_touch"].append(self.body_touch)        
        current_dict["face_on_floor"].append(self.face_on_floor)

        # reset
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

        # count up id 
        self.id += 1

  def callback_sensors(self, sensors):
    self.head_touch += np.sum(np.array(bytearray(sensors.touch_head)).astype(np.float)) 
    self.body_touch += np.sum(np.array(bytearray(sensors.touch_body)).astype(np.float)) 

  def callback_core_state(self, core_state):
    # get current emotional state
    self.emotion_valence = core_state.emotion.valence
    self.core_message.emotion_drive_target.arousal = core_state.emotion.arousal


  def shutdow_hook(self):
    # stop core behavior
    c = core_config()
    c.msg_flags = c.FLAG_UPDATE_SIGNALS
    self.pub_core_config.publish(c)    

  def main(self):
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
      rate.sleep()
      #self.core_message = core_control()
      self.core_message.header.stamp = rospy.Time.now()
      self.pub_core_control.publish(self.core_message)
 
    # try to save the file
    try:
      with open(self.monitor_file, "w") as filehandle:
        for entry_dict in self.monitor_list[1:]:
          for key in entry_dict:
            entry_dict[key] = NoIndent(entry_dict[key])

        filehandle.write(json.dumps(self.monitor_list, cls=NoIndentEncoder, indent=4))

        # json.dump(self.monitor_list, filehandle)
        print("file saved")
    except IOError:
      print("error at saving")


if __name__ == '__main__':
  monitor = monitor()
  monitor.main()
