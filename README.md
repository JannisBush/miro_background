# Documentation MiRo RISE Project at Aberystwyth University Summer 2018
*Jannis Rautenstrauch jrautenstrau@uni-onsbrueck.de*

This is the main repository of the internship - **Monitoring Behaviour with MiRo** - conducted by me at Aberystywth Univeristy in the summer of 2018. In this internship I investigated how a robot like MiRo *[offical website](http://consequentialrobotics.com/miro/)* could be used as a social (health-care) robot to monitor short- and long-term behaviour of people and be a playful companion to them. There are three main branches of research regarding tracking/monitoring of behaviour: smart home (fixed sensors placed in a house), wearable devices (e.g. smart watches) and mobile robots as observers. This project belongs to the third branch. 

Below you will find part of the documentation of my work, as well as installation and usage instructions for different software I used or created. Please read the complete README (and optimally also the other `.md`-files in this repository), before you try to install and start the included software.
If you have any questions, write me a mail or open an issue.

## Installation, requirements and configuration
- Workstation with Ubuntu 16.04 natively installed *Note: I recommend a clean installation (only) used for the work with this robot. A partition with 30gb should easily be enough.*
- Real or simulated MiRo robot *Note: not everything will work with the simulated robot* 
	- [see here to buy the robot or download the simulator](http://consequentialrobotics.com/miro/)
- ROS Kinetic and catkin workspace
	- [installation instructions](http://wiki.ros.org/kinetic/Installation)
	- [configuration and set-up](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
- MDK 180509
	- [download here](http://labs.consequentialrobotics.com/miro/mdk/)
	- [installation instructions](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html)
- Add the following lines to your `~/.bashrc`-file or any other file, which is sourced in every terminal you use and change the IP address to your IP address *Note: you shouldn't use python3 or conda on the machine you use, but only the systems standard python*
```bash
#####ROS + MIRO Stuff
## ROS

# Normal ROS distribution
# source /opt/ros/kinetic/setup.bash

# Own Catkin Workspace (this includes the normal ROS distribution)
source ~/catkin_ws/devel/setup.bash

# Set the editor for rosed (change to your preferred editor)
export EDITOR='nano -w'

## MIRO MDK + Communication with MIRO
export MIRO_PATH_MDK=~/mdk

# Shorcuts to either connect with the real miro or with the simulated MiRo (insert your IP address)
alias realMiro='export ROS_IP=<your_ip>; export ROS_MASTER_URI=http://<your_ip>:11311'
alias simMiro='export ROS_IP=localhost; export ROS_MASTER_URI=http://localhost:11311'
# Default=realMiro (change the default to what you will normally use)
realMiro

# Make our custom messages available to ROS/PYTHON
export ROS_PACKAGE_PATH=$MIRO_PATH_MDK/share:$ROS_PACKAGE_PATH
export PYTHONPATH=$MIRO_PATH_MDK/share:$PYTHONPATH

# Usual Gazebo setup
source /usr/share/gazebo/setup.sh

# Announce MIRO resource to Gazebo
export GAZEBO_RESOURCE_PATH=$MIRO_PATH_MDK/share:${GAZEBO_RESOURCE_PATH}

#####ROS + MIRO end
```
- My version of image_recognition (Branch MiRo) [git](https://github.com/JannisBush/image_recognition/tree/miro)
	- `$ cd ~/catkin_ws/src` and `git clone https://github.com/JannisBush/image_recognition.git` and `$ cd .. && catkin_make`. You also have to switch to branch MiRo `$ cd ~/catkin_ws/src/image_recognition` and `$ git checkout miro`
	- additional:
		- to use some of the subsystems you need to install additional packages
		- follow the installation instructions of the REAME files in the sub-folders: e.g. for tensorflow_ros `sudo apt-get install tensorflow`
		- optional: train with new images to recognize your objects
- My version of ros_posenet  [git](https://github.com/JannisBush/ros_posenet)
	- `$ cd ~/catkin_ws/src`  and `$ git clone https://github.com/JannisBush/ros_posenet.git` and `$ cd .. && catkin_make`
	- if NodeJS8.X is not installed `$ curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -` and `$ sudo apt-get install -y nodejs`
	- `$ cd ~/catkin_ws/src/ros_posenet/` and `$ npm install`
- My version of teleop_twist_keyboard [git](https://github.com/JannisBush/teleop_twist_keyboard_MiRo)
	- `$ cd ~/catkin_ws/src`  and `$ git clone https://github.com/JannisBush/teleop_twist_keyboard_MiRo.git` and `$ cd .. && catkin_make`
- This repository (miro_background)
	- `$ cd ~/catkin_ws/src`  and `$ git clone https://github.com/JannisBush/teleop_twist_keyboard_MiRo.git` and `cd .. && catkin_make`
	- Now, you should be able to run all of the main applications 
- **Optional: also install the following packages to get the full functionality**
	- My version of face_detection [git](https://github.com/JannisBush/face_detection)
		- If you have not yet installed/downloaded dlib, download it [here](http://dlib.net)
		- `$ cd ~/catkin_ws/src` and `$ git clone https://github.com/JannisBush/face_detection.git`
		- Change the path in line 131 of `~/catkin_ws/face_detection/CMakeLists.txt` to where you downloaded dlib to
		- `$ cd ~/catkin_ws && catkin_make`
	- My version of features_face including OpenFace [git](https://github.com/JannisBush/features_face)
		- `$ cd ~/catkin_ws/src` and `$ git clone https://github.com/JannisBush/features_face.git`
		- `$ cd ~/catkin_ws/src/features_face/OpenFace/build` and `$ cmake -D CMAKE_BUILD_TYPE=RELEASE ..` and `$ make` 
		- If there occurs an error like: `error: no type named 'channel_type' in 'cv::DataType<unsigned int>'` or similar, you have to uncomment the line `#define OPENCV_TRAITS_ENABLE_DEPRECATED` in 	`traits.hpp`
			-  `sudo nano /opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/core/traits.hpp`, uncomment the line, close and save and then run `$ make` again
		- Now, the program should work standalone as well as with real_monitor.launch. The only thing you can't do is to listen to the /faces topic with rostopic echo, because ros does not know were the message are saved> You can see the overlay image at `/openface/viz` and if you use `miro_background/real_monitor.launch` or `miro_background/real_camera.launch` you can also get a feel for the recognized emotions. There should be two options to circumvent this problem, but I could not get them to work easily and the problem is not essential
			- Build the OpenFACEROS-messages in a way ros finds them
				- Don't know how to do this
			- Change the messages used in `OpenFace/exe/OpenFaceROS/openface_ros.cpp` from `OpenFaceROS/msgs` to `miro_background/msg`
				- Some scope errors occur?

## Running the programs

#### miro_background [git](https://github.com/JannisBush/miro_background)
Named like this for historic reasons and includes all the main applications I made. 

- Start with `$ roslaunch miro_background <name.launch>`:
	- `real_monitor.launch`: main application including monitoring and reaction to touch, sound (primitive analysis), poses (posenet), emotions (OpenFace), horizontal faces (dlib), objects (tensorflow/inception)
		- miro_background/monitor_all.py
		- miro_background/face_safety and face_detection/face_detection_dlib
		- miro_background/recognize_objects.py and image_recognition/tensorflow_ros/object_recognition_node
		- 2x image_transport/republish
		- ros_posenet/posenet.js (only works with a internet connection, because the model has to be downloaded first)
		- miro_background/miro_camera.py and feature_faces/exectuable_launcher_op.py (OpenFace)
	- `real_camaera.launch`: older version of emotion recognition standalone
		- miro_background/openface_listener.py 
		- miro_background/miro_camera.py
		- feature_faces/exectuable_launcher_op.py (OpenFace)
		- 1x image_transport/republish
		- image_view/image_view
		- additional: to analyse the output run: `roscd miro_background`, then `other_scripts/check_emotions.sh ~/.ros/log/latest/miro_openface-2-stdout.log; other_scripts/check_actions.sh ~/.ros/log/latest/miro_openface-2-stdout.log` *Note: you may have to replace the 2 by another number, press `tab` twice to get a hint*
	- `real.launch`: old idea of some background safety nodes including teleoperation
		- miro_background/miro_teleop.py
		- miro_background/sensor_to_pos.py
		- miro_background/face_safety.py and face_detection/face_detection_dlib
		- miro_background/background_safety.py
		- 1x image_transport/republish
	- `sim.launch`: old version of real.launch with the simulated MiRo
- (Standalone applications) Start with `$ rosrun miro_background <name_of_script> [<additional options>]`:
	- `miro_teleop.py robot=(rob01|sim01)`: receiver node for teleoperation 
		- Needs a publisher node in addition: start with `$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
	- `sensor_to_pos.py robot=(rob01|sim01)`: publishes odometry and tf of MiRo (can be used to try mapping + visualization in rviz)
	- `recorded_sounds_random_lights.py robot=(rob01|sim01)`: can play back all the prepacked sounds on MiRo's P1 and P2 and also includes a random light generator
		- Send `$rostopic pub /soundP[12] std_msgs/Int8 1` to start a loop over all available P1 or P2 sounds (Bug of MiRo: there are only ~30 sounds availabe, but in the end it will loop over the last 4? sounds)
		- Send `$ rostopic pub /soundP3 std_msgs/Int8 <Number>` to play the sound `Number` on P3 (default Number between 1-5, 0 to do nothing and -1 to stop the sound)
		- Send `$ rostopic pub -r <hz> /lights String a` to let the robot randomly flash it's leds `<hz>` times per second
	- `sound_follow_1_2.py robot=(sim01|rob01) follow_type=(mics|core)`: two options of following a sound source, one using the spatial interface the other using just the crosscorrelation between the two microphones (used in real_monitor.launch/monitor_all.py)
	- `audio_display.py`: displays the audio signal and has some ideas for primative sound analysis (only for the real robot)
	- `recognize_objects.py`: categorizes the toys (only works with the real robot)
		- Needs the recognition server running: start with `$ rosrun tensorflow_ros object_recognition_node _graph_path:=<path_to_graph.pb> _labels_path:=<path_to_labels.txt>`
	- `platform_control_server.py` and `control_test.py`: ideas and test for using services instead of publisher/subscriber (topics) (only works with the real robot and you have to start both)

#### ros_posenet [git](https://github.com/JannisBush/ros_posenet)
Package to perform human pose estimation using posenet/tensoflor.js. 
Needs an internet connection at the start, because the model is not saved locally and will get downloaded at the start. 

- To start the pose estimation run: `$ roslaunch ros_posenet posenet.launch` (you can change the camera/robot in the launch file)
	- ros_posenet/posenet.js 
	- image_transport/republish
- To display the detected keypoints as a skeleton run: `$ rosrun ros_posenet display_keypoints.py`
	- display_keypoints.py: displays the keypoints (skeleton), can be used with posenet.launch or real_monitor.launch

#### teleop_twist_keyboard_MiRo [git](https://github.com/JannisBush/teleop_twist_keyboard_MiRo)
Package to move the robot around using the keyboard, also includes the possibility to move the head.

- Run with `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

#### image_recognition [git](https://github.com/JannisBush/image_recognition/tree/miro)
Package including a lot of image_recognition tools (some of them are used in the main application)

- `$ rosrun image_recognition_rqt <name>`
	- `folder_image_publisher`: you can use it to publish some images saved in a folder to ROS
	- `annotation_gui`: you can use it to annotate objects
	- `test_gui`: you can use it to test the recognition of the objects
- `$ rosrun tensorflow_ros <name>` (needs tensorflow installed)
	- `retrain [iamge_folder] [model_folder_inceptionv3] [output_dir]`: to train the classifier on the annotated images/objects
	- `object_recognition_node _graph_path:=<path_to_graph.pb> _labels_path:=<path_to_labels.txt>`: to start the recognition service
	- for other uses see the original github repo
- `$ rosrun openface_ros <name>` (needs openface -not OpenFace- installed, see original github repo for installation instructions)
	- `face_recognition_node`: to train on faces and start the recognition service
- `$ rosrun keras_ros <name>` Age and gender estimation (needs keras installed and models downloaded, see original github repo for instructions)
	- `face_properties_node _weights_file_path:=<path_to_model>`: to start the recognition service
- For more uses (pose recognition, skybiometry) look at the original repo

#### face_detection [git](https://github.com/JannisBush/face_detection)
Package to detect faces using different approaches. (face_detection_dlib is used in the main application) See [here](http://wiki.ros.org/face_detection_tracking) for further uses.
 
 - `$ roslaunch face_detection face_detection[_tracking|_dlib].launch`
 - optional `$ rosrun rqt_reconfigure rqt_reconfigure` to change parameters and configuration, e.g. which image topic to use
 - `$ rosrun face_detection face_listener`

#### features_face or OpenFace [git](https://github.com/JannisBush/features_face)
Package to detect human emotions/Facial Action Units. 

- `$ roslaunch features_face openface_ros.launch`
- Optional `rosrun image_view image_view image=/openface/viz` to see the landmark detection

## How to prepare and use Sim/Real Miro
- To  use a simulated MiRo:
	- you have to configure ROS_IP, etc. for local use for every terminal you use: the easiest way is to run `$ simMiRo` as the first command in every opened terminal (if you only use the sim and not the real robot, change the default in `~/.bashrc`)
	- open a terminal and run `$ roscore`
	- open a second terminal and run `$ cd ~/mdk/sim/gazebo; ./launch_sim.sh`
	- open N other terminals to run the applications you want
	- see an example [here](https://consequential.bitbucket.io/Developer_Profiles_Simulation.html)
- To use a real MiRo:
	- first commission your MiRO [see here for details](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html)
	- configure MiRo's settings: `$ ssh root@<MIRO_IP>`, then `$ nano ~/.profile` and change the *ROS_IP* to *<MIRO_IP>* and *ROS_MASTER_IP* to your workstations IP
	- open a terminal and run `roscore`
	- open a second terminal and connect to MiRO `$ ssh root@<MIRO_IP>`, then run `$ run_bridge_ros.sh` there
	- open N other terminals on your workstation and run the applications you want 
	- *Note: This is the Off-board profile*
- Note on using the Core-Interface (Demo/Autonomous life):
	- the core behavior specified in core_config only works if you send control messages at at least 1hz to MiRo, not ALL messages will work, e.g. core_config messages won't work, the safest bet is to send empty core_control messages (if the drive is 0, they don't do anything).
- Note on using applications and publishing:
	- always make sure to close every other application before you start a new one, or at least that there is only one publisher for every interface of MiRo, because otherwise there will be a race condition between the nodes/publishers and nothing will work as expected.
- Note on creating new applications:
	- You can either write applications for MiRo in Python or in C++. Here, I mainly talk about Python applications. Please see [here](https://consequential.bitbucket.io/Developer_Examples.html) for general examples from Consequential Robotics (including a C++ application). Every Python application must have the following line `#!/usr/bin/env python` as the first line and has to be marked as executable `$ chmod +x <name_of_file>`
- Note on timeStamp:
	- Up to MDK 180509 there is an error in the timeStamp of MiRo's messages. The time jumps from X536.8 seconds to (X+1)000 seconds. The easiest workaround is to just use the time of the workstation instead.
- Note on image streams:
	- You should always use the compressed image stream, because the uncompressed stream can lag behind and generates high network loads.
	- If you need an uncompressed stream (e.g. for ros_posenet), decompress on your workstation and republish using `rosrun image_transport republish compressed in:=/miro/rob01/platform/caml raw out:=/uncompressedWithoutLag/caml` or similar

## Longer explanation about the main application etc.

#### real_monitor.launch or monitor_all.py
The main application monitors many different features, which MiRo can detect, reacts to them and outputs a human-readable log-file at the end. 

- To start the application run `$ roslaunch miro_background real_monitor.launch`. 
- There are many parameters in `real_monitor.launch`, you might want to change them or have a look at them
	- General:
		- `monitor_emotions` and `alert_faces`: these activate/deactivate the usage of the emotional recognition using features_face/OpenFace and face_detection/face_detecion_dlib. You have to set them to false, if you have not installed the correct software.
	- regarding `monitor_all.py`:
		- `robot_name`: the name of the robot *Note: only changing this is not enough, if you want to use another robot. You also have to change the image topics used*
		- `start_wait`: how manys seconds the program waits, before the monitoring/reactions start
		- `interval_time`: how many seconds on interval should have
		- `days`: how many "days" you want to run the application
		- `intervals_per_day`: how many intervals a day consists of
		- `demo_mode`: if the autonomous life should be switched on
		- `demo_move`: if the movement in the autonomous life should be used (only works if `demo_mode` is True)
		- `monitor_file`: the base name of the output monitor file
		- `min_part_conf`: the minimal confidence score to accept a keypoint of posenet 
	- regarding `face_safety.py` and `face_detection_dlib` (only if `alert_faces` is True):
		- Which cascades to use (the default ones should be fine)
	- regarding `object_recogniton_node` and `recognize_objects.py`:
		- `_graph_path`: which graph/trained network to use
		- `_labels_path`: path for the labels for the graph
		- `topic_left`: name of the left image stream (should not be changed here, but in image_transport)
		- `topic_right`: same
	- regarding `image_transport/republish`:
		- `in`: name of the input camera stream (left or right)
		- `out`: name of the output camera stream (should not be changed)
	- regarding `posenet.js`:
		- see the launch file for documentation
	- regarding `feature_faces` and `OpenFace`:
		- see the launch file for documentation
- The program will run for `interval_time` * `days` * `intervals_per_day` seconds and monitor everything in the same time-interval structure (counting up during on interval and reset the counters at the start of the next interval)
- The program will also (appropriatly) react to all the things it can recognize, there is some "gaussian" noise over the reactions, so the robot won't do exactely the same thing all the time
	- object/toys *You can hold the object in front of his head (camera), which camera does not matter, because both cameras send 1 image per second to the recognition service*:
		- The robot will approach toys he likes *default: ball_g (green ball), ball_rb (red and blue ball), moto5 (Lenovo Moto G5*
		- The robot will back away from toys he dislikes *default: ball_r (red ball), screwdriver*
		- The robot will turn to objects which a interesting/strange1 *default: ball_b, ball_y, purse*
		- The robot will turn away from disgusting/strange2 objects *default: pen (white pen), ribena (500ml ribena bottle)*
	- touch:
		- the robot uses his autonomous behaviour reacting to head and body touch, this depends on the current mood of MiRo
	- sounds:
		- the robot will drop his tail, if he hears a sound he hates *Android BeeBeep-Alarm*
		- the robot will move his ears, if he hears another bad sound *Android Beep-Beep-Beep-Alarm*
		- the robot will approach you, if you clap fast and loud *~3 times per second clapping*
		- if the autonomous life is on, the robot will also react to all king of sounds and this can be a bit contradicting, e.g. backing away from loud sounds
	- human poses *this uses the left camera only*:
		- if you hold both your hands up, i.e. both wrists are above your shoulders, the robot will make sounds *you have to walk the plank*
		- if your left knee is higher than your left hip, the robot will make laughing sounds
		- if your right knee is higher than  your right hip, the robot will make random pirate comments
	- xtra:
		- light: there is only the reaction of the autonomous behaviour
		- face_on_floor: if the robot will detect a horizontal face with his left camera, he will beep and blink red
	- emotions:
		- he will move his cosmetic joints to a happy state, if he recognizes a happy face and he will move his cosmetic joints to a sad state, if he recognizes a sad face
		- there is no direct reaction to the other emotions and facial action he can recognize
- The program will use a "decision tree"-like approach to map from his observations to a name/label of that interval
- If you start the program again using the same parameters, the program will load the old log file and append the new monitoring information 
- The program will try to predict the name/label of an interval at the start. It will try to find unique most common label for this interval of the past, if there is none it predicts "neutral" and will then use a basic mapping from the predicted name/label to MiRo's internal mood/emotion/sleep state and set it accordingly to activate the indirect reactions
	- I created 6 distinct internal states. The state is made up out of Emotion-valence, Emotion-arousal, Mood-valence, Mood-arousal, Sleep-wakefulness, Sleep-pressure. Emotion and Mood is always the same. Therefore we get: Neutral (0.5,0.5,1.0,0.0), Angry Disturbed (0.0,1.0,1.0,0.0), Angry Calm (0.0,0.0,1.0,0.0), Happy Active (1.0,1.0,1.0,0.0), Happy Calm (1.0,0.0,1.0,0.0) and Sleep (0.5,0.5,0.0,1.0)
	- e.g. in the second interval nothing is happening all the time, MiRo will sleep in that interval
	- it will only set the internal state at the start of the interval, afterwards it will change autonomously according to MiRo's surrounding *e.g. touch -> increases happiness, noise -> decreases happiness*
- A nice improvment for the future would be to produce a labelled dataset and replace the mapping from observations to names/labels to internal states by some classification from observations to labels

## Additional Information
- I also tested mapping using another robot (TurlteBot 3 Burger). See [here](relative_path) for documentation.
- I also tested, used, created some other software and read many papers. See [here](relative_path) for describtions, links and instructions.
