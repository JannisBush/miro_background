#!/usr/bin/env python
import uuid
import json
import copy

import numpy as np
import rospy
from collections import Counter

from miro_constants import miro
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray, UInt16MultiArray, Int8
from geometry_msgs.msg import Twist
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, \
    platform_control, core_state, core_control, core_config, bridge_config, bridge_stream
from ros_posenet.msg import Poses, Pose, Keypoint
from miro_background.msg import Face, Faces, ActionUnit, ObjectSide


#### just needed for a human readable format of the log file ############################
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
            # if the key is one of these, don't indent the sublevel of these
            if k in ["object/toys", "touch", "sounds", "face/emotions", "xtra", "poses"]:
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
    # or do nothing, i.e indent using the standard rules
    else:
        pass
########################################################################################


class monitor:
    """Class to use MiRo to monitor how people interact with him."""
    def __init__(self):
        """Prepares MiRo

        E.g. setting all neccessary starting conditions and read the old log file
        """   
        # Get all parameters from the launch file
        self.robot_name = rospy.get_param("~robot_name")
        self.demo_mode = rospy.get_param("~demo_mode")
        self.demo_move = rospy.get_param("~demo_move")
        self.start_wait = rospy.get_param("~start_wait")
        self.interval_time = rospy.get_param("~interval_time")
        self.days = rospy.get_param("~days")
        self.hours_per_day = rospy.get_param("~intervals_per_day")
        self.monitor_emotions = rospy.get_param("~monitor_emotions")
        self.monitor_file = "%s_%s_%s_%s_%s.log" % (rospy.get_param("~monitor_file"), str(self.days), str(self.hours_per_day), str(self.interval_time), str(self.monitor_emotions))
        self.min_part_conf = rospy.get_param("~min_part_conf")
        self.monitor_emotions = rospy.get_param("~monitor_emotions")

        # Prepare the dictionaries for the log file
        # There are 5 different categories of things MiRo monitors + one uncategorized category
        self.toy_template = {"ball_b": [], "ball_y": [], "ball_r": [],
         "ball_g": [], "ball_rb": [], "screwdriver": [], "pen": [], "ribena": [], "moto5": [], "purse": []}
        self.touch_template = {"head_touch": [], "body_touch": []}
        self.emotion_template = {"happiness": [], "sadness": [], "blink": [], "surprise": [],
         "fear": [], "anger": [], "disgust": [], "contempt": []}
        self.sound_template = {"beep": [], "beep_beep": [], "clap": []}
        self.pose_template = {"hands_up": [], "high_knee_left": [], "high_knee_right": []}
        self.extra_template = {"face_on_floor": [], "light": []}
        # The event template consists of the 5+1 categories (deepcopy, to create new lists)
        self.event_template = {"object/toys": copy.deepcopy(self.toy_template), "touch": copy.deepcopy(self.touch_template),
         "face/emotions": copy.deepcopy(self.emotion_template), "sounds": copy.deepcopy(self.sound_template),
         "poses": copy.deepcopy(self.pose_template),"xtra": self.extra_template}
        
        # Prepare the other templates for hours and days
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

        # Add a shutdown hook, to savely close the program (stop core behavior at shutdown)
        rospy.on_shutdown(self.shutdow_hook)

        # try to load the file 
        try:
            with open(self.monitor_file, "r") as filehandle:
                print("load the file")
                self.monitor_list = json.load(filehandle)
                # Increment the number of runs by one 
                self.col_id = self.monitor_list[0] + 1
                self.monitor_list[0] = self.col_id
        # if the file does not exist, create a new file
        except IOError:
            print("create a new file")
            self.col_id = 0
            self.monitor_list = [self.col_id, [copy.deepcopy(self.day_template) for _ in range(self.days)]]

        # Set all counters to 0, all 6 categories are in one dict to make it easier
        self.counter_dict = {}
        for topic_name in self.event_template:
            for content_name in self.event_template[topic_name]:
                self.counter_dict[content_name] = 0

        # Prepare miro's topic
        topic_root = "/miro/" + self.robot_name

        # Set up all core publishers
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control", core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)

        # Wait the time specified in launch file. This is necessary, because the publishers may not be ready immediately 
        rospy.sleep(self.start_wait)
        # After the waiting time, set the starting time
        self.start_time = rospy.Time.now().to_sec()

        # Set the dict to map from patterns to states
        self.pattern_to_state_dict = {"nothing": "Sleep", "face_on_floor": "Angry_disturbed", "happy_face": "Happy_active",
         "body_touch": "Happy_active", "head_touch": "Happy_calm", "sad_face": "Angry_calm", "liked": "Happy_active",
         "disliked": "Angry_disturbed", "strange1": "Neutral", "strange2": "Angry_calm", "sound": "Angry_disturbed",
         "hands_up": "Happy_active", "neutral": "Neutral"}
        # Set the dict to map from states to /core/control messages 
        self.state_dict = {"Neutral": [0.5, 0.5, 0.5, 0.5, 1.0, 0.0], "Angry_disturbed": [0.0, 1.0, 0.0, 1.0, 1.0, 0.0], 
        "Angry_calm": [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], "Happy_active": [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
        "Happy_calm": [1.0, 0.0, 1.0, 0.0, 1.0, 0.0], "Sleep": [0.5, 0.0, 0.5, 0.0, 0.0, 1.0]}

        # If the flag to use miro's demo behavior was set, activate the demo behavior
        # This is almost the same as the normal demo mode in the gui application
        # The expression through ears is deactivated, because otherwise the ears move alot
        # And you have the option to activate or deactivate the move behavior 
        if self.demo_mode:
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
            # Only activate demo movement if configured in the launch file
            if self.demo_move:
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
     
        # Add all other publishers
        # Platform control (play a sound as a reaction)
        self.joint_state = None
        self.eyelid_closure = 0.0
        self.platform_control_pub = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
        self.stop_sound_pub = rospy.Publisher("/stop/sound", Int8, queue_size=1)
        self.stop_sound_sub = rospy.Subscriber("/stop/sound", Int8, self.callback_stop_sound, queue_size=1)
        # Cmd_vel (move as a reaction)
        self.move_pub = rospy.Publisher(topic_root + "/control/cmd_vel", Twist, queue_size=0)
        # Cosmetic joints (close eyes, etc. as reaction)
        self.cosmetic_pub = rospy.Publisher(topic_root + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
        # Lights (display lights as reaction)
        self.lights_pub = rospy.Publisher(topic_root + "/control/lights", UInt16MultiArray, queue_size=0)

        # Add all subscribers
        # Core state, main logic + sleep state
        self.awake = True
        self.core_state = rospy.Subscriber(topic_root + "/core/state", core_state, self.callback_state, queue_size=1)
        # Face on the floor
        self.face_sub = rospy.Subscriber("/faceCoord", Int32MultiArray, self.callback_faces)
        # Face emotions
        if self.monitor_emotions:
            self.face_sub = rospy.Subscriber("/faces", Faces, self.callback_emotions)
        # Platform sensors (used for Touch)
        self.sub_platform_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_sensors)
        # Poses (PoseNet Human Pose Analysis) -> HandsUp and HighKnees
        self.keypoint_sub = rospy.Subscriber("/poses", Poses, self.callback_poses)
        # Toy recognition
        self.toys_sub = rospy.Subscriber("/toys", ObjectSide, self.callback_toys)
        # Audio recogniton (Beep, BeepBeep and Clap) be aware of false positives
        self.i = 0
        self.max_count = 10
        self.mic_array_1 = np.zeros(2000 * self.max_count)
        self.mic_array_2 = np.zeros(2000 * self.max_count)
        self.mics_sub = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_mics)

        # Predict the pattern for the first time interval
        self.predict_next_pattern()

        # stop all sounds (else it could happen that some old sounds from last time, start playing again)
        rospy.sleep(1)
        self.stop_sound_pub.publish(1)

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
                    print("blink detected") 
                    au45 = True
                    # Increment the blink counter
                    self.counter_dict["blink"] += 1

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
                self.counter_dict["happiness"] += 1
                # react with maxmimal aroused cosmetic joints
                reaction = Float32MultiArray()
                reaction.data = [np.random.uniform(0.8,1),np.random.uniform(0.8,1),np.random.uniform(0.8,1),np.random.uniform(0.8,1)]
                self.cosmetic_pub.publish(reaction)

            # Sadness 1 + 4 + 15
            if au1 is True and au4 is True and au15 is True:
                print("sadness detected")
                self.counter_dict["sadness"] += 1
                # react with minimal aroused cosmetic joints
                reaction = Float32MultiArray()
                reaction.data = [np.random.uniform(0,0.2),np.random.uniform(0,0.2),np.random.uniform(0,0.2),np.random.uniform(-1,-0.8)]
                self.cosmetic_pub.publish(reaction)

            # Surprise 1 + 2 + 5 + 26 
            if au1 is True and au2 is True and au5 is True and au26 is True:
                print("surprise detected")
                self.counter_dict["surprise"] += 1
            
            # Fear 1 + 2 + 4 + 5 + 7 + 20 + 26
            if au1 is True and au2 is True and au4 is True and au5 is True and au7 is True and au20 is True and au26 is True:
                print("fear detected")
                self.counter_dict["fear"] += 1

            # Anger 4 + 5 + 7 + 23
            if au4 is True and au5 is True and au7 is True and au23 is True:
                print("anger detected")
                self.counter_dict["anger"] += 1

            # Disgust 9 + 15 (+16 not recongized, therefore ignored)
            if au9 is True and au15 is True:
                print("disgust detected")
                self.counter_dict["disgust"] += 1

            # Contempt 12 + 14 (on one side of the face only, buy this implementation can't distinguish this, therefore more like another happiness)
            if au12 is True and au14 is True:
                print("contempt detected")
                self.counter_dict["contempt"] += 1

    def callback_faces(self, face_array):
        """Listen to the face on the floor message"""
        # Do nothing if the monitor is not yet active
        if not self.active:
            return

        # Increment the counter, if a face was detected 
        if face_array.data[1] != 0:
                self.counter_dict["face_on_floor"] += 1


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
        if state.sleep.wakefulness < 0.1:
            self.awake = False
        else:
            self.awake = True

        # Check if the next interval has started 
        if rospy.Time.now().to_sec() > self.start_time + (self.current_day * self.hours_per_day + self.current_hour + 1) * self.interval_time:

            # Deactivate logging while recording takes place (against lost update, dirty read, etc.)
            self.active = False
         
            # Get the corresponding event dict
            current_event_dict = self.current_hour_dict["events"]
            # Log all basic units
            for content_name in self.counter_dict:
                for topic_name in current_event_dict:
                    if content_name in current_event_dict[topic_name]:
                        current_event_dict[topic_name][content_name].append(self.counter_dict[content_name])

            # toy with the highest number
            max_toy = max(current_event_dict["object/toys"], key=lambda k: current_event_dict["object/toys"][k])
            # Which pattern was actually observed?
            # Was there a face on the floor? (highest priority 1)
            if self.counter_dict["face_on_floor"] > 0:
                observed_pattern = "face_on_floor"
            # Were happy or sad faces present? (middle priority 2)
            elif (self.counter_dict["happiness"] + self.counter_dict["sadness"]) > 0:
                #if self.happiness > self.sadness:
                if self.counter_dict["happiness"] > self.counter_dict["sadness"]:
                    observed_pattern = "happy_face"
                else:
                    observed_pattern = "sad_face"
            # Were toys present? (middle priority 3)
            elif (current_event_dict["object/toys"][max_toy][-1] > 0):
                observed_pattern = self.toys_dict[max_toy]
            # Were sounds present (middle priority 4)
            elif (any(self.counter_dict[k] > 0 for k in self.sound_template)):
                observed_pattern = "sound"
            # HandsUP? (middle priortiy 5)
            elif self.counter_dict["hands_up"] > 0:
                observed_pattern = "hands_up"
            # Was miro touched? (lowest priorioty 6)
            elif (self.counter_dict["body_touch"] + self.counter_dict["head_touch"]) > 0:
                #if self.body_touch > self.head_touch:
                if self.counter_dict["body_touch"] > self.counter_dict["head_touch"]:
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
                

            # Reset all counters
            for k in self.counter_dict:
                self.counter_dict[k] = 0

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
        
        #activate logging (again)
        self.active = True  

    def callback_sensors(self, sensors):
        """Listen to the platform/sensor messages to detect touch."""
        # Do nothing if the monitor is not yet active
        if not self.active:
            return
        # Increment head and body touch if touched
        self.counter_dict["head_touch"] += np.sum(np.array(bytearray(sensors.touch_head)).astype(np.float))
        self.counter_dict["body_touch"] += np.sum(np.array(bytearray(sensors.touch_body)).astype(np.float))
        # Increment counters for the light sensors
        self.counter_dict["light"] += np.sum(np.array(bytearray(sensors.light)).astype(np.float))
        # Save the current state of the joints and eyelids (to not change it)
        self.joint_state = sensors.joint_state
        self.eyelid_closure = sensors.eyelid_closure

    def callback_poses(self, pose_msg):
        """Listens to poses messages to detect human poses (e.g. HandsUp)."""
        # Do nothing if the monitor is not yet active
        if not self.active:
            return
        # Detect the human poses for all incoming pose messages
        for i in range(len(pose_msg.poses)):
            keypoint_dict = {}
            for keypoint in pose_msg.poses[i].keypoints:
                if keypoint.score > self.min_part_conf:
                    keypoint_dict[keypoint.part] = (int(keypoint.position.x), int(keypoint.position.y))

            # information for skeleton and detection
            connected_part_names = [
            ["leftHip", "leftShoulder"], ["leftElbow", "leftShoulder"],
            ["leftElbow", "leftWrist"], ["leftHip", "leftKnee"],
            ["leftKnee", "leftAnkle"], ["rightHip", "rightShoulder"],
            ["rightElbow", "rightShoulder"], ["rightElbow", "rightWrist"],
            ["rightHip", "rightKnee"], ["rightKnee", "rightAnkle"],
            ["leftShoulder", "rightShoulder"], ["leftHip", "rightHip"]]
     
            # detect hands up
            if all(k in keypoint_dict for k in ("rightWrist", "rightShoulder", "leftWrist", "leftShoulder")):
                if keypoint_dict["rightWrist"][1] < keypoint_dict["rightShoulder"][1] and keypoint_dict["leftWrist"][1] < keypoint_dict["rightShoulder"][1]:
                    # hands up detected
                    self.counter_dict["hands_up"] += 1
                    # react with sound 
                    reaction = platform_control()
                    reaction.eyelid_closure = self.eyelid_closure
                    reaction.body_config = self.joint_state.position
                    # random aggressive pirate sound
                    reaction.sound_index_P2 = np.random.choice([9,10,23])
                    if self.awake:
                        self.platform_control_pub.publish(reaction)
                    # stop the sound after 1 second
                    self.stop_sound_pub.publish(1)

            # detect high knees right
            if all(k in keypoint_dict for k in ("rightHip", "rightKnee")):
                if keypoint_dict["rightKnee"][1] < keypoint_dict["rightHip"][1] :
                    # high knee right detected
                    self.counter_dict["high_knee_right"] += 1
                    # react with sound 
                    reaction = platform_control()
                    reaction.eyelid_closure = self.eyelid_closure
                    reaction.body_config = self.joint_state.position
                    # random neutral pirate sound
                    reaction.sound_index_P2 = np.random.choice([1,3,5,6,7,8])
                    if self.awake:
                        self.platform_control_pub.publish(reaction)
                    # stop the sound after 1 second
                    self.stop_sound_pub.publish(1)

            # detect high knees left
            if all(k in keypoint_dict for k in ("leftHip", "leftKnee")):
                if keypoint_dict["leftKnee"][1] < keypoint_dict["leftHip"][1] :
                    # high knee left detected
                    self.counter_dict["high_knee_left"] += 1
                    # react with sound 
                    reaction = platform_control()
                    reaction.eyelid_closure = self.eyelid_closure
                    reaction.body_config = self.joint_state.position
                    # random pirate laugh
                    reaction.sound_index_P2 = np.random.choice([11,12,13,14,15,16,17,18,19])
                    if self.awake:
                        self.platform_control_pub.publish(reaction)
                    # stop the sound after 1 second
                    self.stop_sound_pub.publish(1)

    def callback_mics(self, data):
        """Listens to the microphones to detect specific sounds."""
        # Do nothing if the monitor is not yet active
        if not self.active:
            return
        # Forget the oldest message
        self.mic_array_1 = np.roll(self.mic_array_1, -2000)
        self.mic_array_2 = np.roll(self.mic_array_2, -2000)
        # Add the newest message (split up both microphones)
        self.mic_array_1[(self.max_count-1)*2000:] = data.data[::2]
        self.mic_array_2[(self.max_count-1)*2000:] = data.data[1::2]
        # Every one second, analyse
        if self.i == 0:
            # print("Last second:")
            hist_mat = np.ones(shape=(10,3))
            for i in range(10):
                hist = np.histogram(np.abs(self.mic_array_1[i*2000:(i+1)*2000]), bins=[0,1,5,10,50,500,2000])[0]
                # print(hist[3:])
                hist_mat[i] = hist[3:]

            large = np.count_nonzero(hist_mat[:,2])
            middle = np.count_nonzero(hist_mat[:,1])
            small = np.count_nonzero(hist_mat[:,0])

            if small == 10 and middle == 10 and large < 2:
                # print("Beep!")
                self.counter_dict["beep"] += 1
                # react with one ear aroused and dropped tail
                reaction = Float32MultiArray()
                reaction.data = [np.random.uniform(0.8,1),np.random.uniform(0,0.2),np.random.uniform(0,0.2),np.random.uniform(-1,-0.8)]
                if self.awake:
                    self.cosmetic_pub.publish(reaction)
            elif 3 <= small <= 5 and 3 <= middle <= 5 and large < 2:
                # print("BeepBeep!")
                self.counter_dict["beep_beep"] += 1
                # react with neutral tail and other ear aroused
                reaction = Float32MultiArray()
                reaction.data = [np.random.uniform(0,0.2),np.random.uniform(0.8,1),np.random.uniform(0,0.2),np.random.uniform(-0.1,0.1)]
                if self.awake:
                    self.cosmetic_pub.publish(reaction)
            elif 2 <= middle <= 5 and 2 <= large <= 3:
                # print("Clap!")
                self.counter_dict["clap"] += 1
                # react with approaching 
                reaction = Twist()
                reaction.linear.x = 0.15 + np.random.normal(0.0, 0.05)
                # get location of sound source
                corr = np.correlate(self.mic_array_1[(self.max_count-1)*2000:],self.mic_array_2[(self.max_count-1)*2000:],"same")
                max_corr = np.max(corr[994:1006])
                max_index = np.argwhere(corr == max_corr)[0][0]
                # move to sound source
                max_index = max_index - 1000
                reaction.angular.z = -0.5 * max_index
                if self.awake:
                    self.move_pub.publish(reaction)
        # time increasesS       
        self.i = (self.i+1) % 10

    def callback_toys(self, object_side):
        """Listen to the toy messages to see what toy was detected with which camera."""
        # Do nothing if the monitor is not yet active
        if not self.active:
            return
        # Increment the count for this object
        self.counter_dict[object_side.object] += 1
        # toys dictionary
        self.toys_dict = {"ball_g": "liked", "ball_r": "disliked", "ball_b": "strange1", "ball_y": "strange1", "ball_rb": "liked",
        "moto5": "liked", "screwdriver": "disliked", "pen": "strange2", "purse": "strange1", "ribena": "strange2"}
        # reaction dict, (linear.x, angular.z)
        reaction_dict = {"liked": (0.15, 0), "disliked": (-0.15, 0), "strange1": (0, 0.5), "strange2": (0, -0.5)}
        # move forward, backwards, turn depending on the object
        reaction = Twist()
        move_tuple = reaction_dict[self.toys_dict[object_side.object]]
        reaction.linear.x = move_tuple[0] + np.random.normal(0.0, 0.05)
        # turn to strange1 obejcts, turn away from strange2 objects
        if object_side.side == "left":
            reaction.angular.z = move_tuple[1] + np.random.normal(0.0, 0.05)
        elif object_side.side == "right":
            reaction.angular.z = -move_tuple[1] + np.random.normal(0.0, 0.05)
        else:
            reaction.angular.z = 0
        if self.awake:
            self.move_pub.publish(reaction)

    def callback_stop_sound(self, duration):
        """This method breaks the sound loop"""
        rospy.sleep(duration.data)
        # first (some) stop messages get ignored, therefore send several in a short amount of time
        for _ in range(5):
            rospy.sleep(0.05)
            stop_msg = platform_control()
            stop_msg.eyelid_closure = self.eyelid_closure
            stop_msg.body_config = self.joint_state.position
            stop_msg.sound_index_P2 = 0
            self.platform_control_pub.publish(stop_msg)

    def shutdow_hook(self):
        """Stops the core behavior and saves the file, when the program is stopped"""
        # Stop core behavior
        c = core_config()
        c.msg_flags = c.FLAG_UPDATE_SIGNALS
        self.pub_core_config.publish(c) 
        # Set MiRo's joints to reset
        q = platform_control()
        q.body_config_speed = [-1,-1,-1,-1]
        self.platform_control_pub.publish(q)
        # Save only if a full week has passed 
        if self.end == True:
            # Try to save the file
            try:
                with open(self.monitor_file, "w") as filehandle:
                    # Change the indentation for a nice humanreadable format
                    change_indent(self.monitor_list)
                    print(self.monitor_list)
                    # Save the file
                    filehandle.write(json.dumps(self.monitor_list, cls=NoIndentEncoder, indent=4))
                    print("file saved")
            except IOError:
                print("error at saving")     

    def main(self):
        """Main loop publishes the core/control message 10 times per second"""
        rate = rospy.Rate(10) #10hz
        while not rospy.is_shutdown():
            rate.sleep()
            # Publish the core/control message
            self.publish_state()
         
if __name__ == "__main__":
    # First register the node
    rospy.init_node("monitor")

    # Create and run the monitor class
    monitor = monitor()
    monitor.main()
