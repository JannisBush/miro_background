
# Installation

## Requirements
- MiRo robot (real or sim, not everything will work with a simulated MiRo)
- ROS Kinetic
- MDK 180509
- my version of face_detection (changes in config, + compatible with OpenCV3 instead of 2) [git](https://github.com/JannisBush/face_detection)
	- follow installation instructions there
	- additional: change path to dlib in CMakeLists.txt!
- my version of features_face including OpenFace [git](https://github.com/JannisBush/features_face)
	- TODO: create installation instructions
	- additional: change a lot of paths etc.
- my version of image_recognition Branch MiRo [git](https://github.com/JannisBush/image_recognition/tree/miro)
	- follow installation instructions there
	- optional: train with new images to recognize your objects
- my version of ros_posenet  [git](https://github.com/JannisBush/ros_posenet)
	- follow installation instructions there
- my version of teleop_twist_keyboard [git](https://github.com/JannisBush/teleop_twist_keyboard_MiRo)
	- follow installation instructions there
- this repo with all kind of stuff
- loads of python and ROS packages?, etc.

## Scripts in miro_background (run using roslaunch or rosrun)
- roslaunch:
	- real_monitor.launch: main application including monitoring and reaction to touch, sound (primitive analysis), poses (posenet), emotions (OpenFace), horizontal faces (dlib), objects (tensorflow/inception)
		- miro_background/monitor_all.py
		- miro_background/face_safety and face_detection/face_detection_dlib
		- miro_background/recognize_objects.py and image_recognition/tensorflow_ros/object_recognition_node
		- 2x image_transport/republish
		- ros_posenet/posenet.js
		- miro_background/miro_camera.py and feature_faces/exectuable_launcher.py (OpenFace)
	- real_camaera.launch: old version of emotion recognition standalone
		- miro_background/openface_listener.py 
		- miro_background/miro_camera.py
		- feature_faces/exectuable_launcher.py (OpenFace)
		- 1x image_transport/republish
		- image_view/image_view
		- additional: use check_actions.sh or check_emotions.sh to control the output
	- real.launch: old idea of some background safety nodes including teleop
		- miro_background/miro_teleop.py
		- miro_background/sensor_to_pos.py
		- miro_background/face_safety.py and face_detection/face_detection_dlib
		- miro_background/background_safety.py
		- 1x image_transport/republish
	- sim.launch: old version of real.launch with the simulated MiRo
- (Standalone) rosrun:
	- (robot=sim01|rob01) miro_teleop.py: receiver node for teleoperation
	- (monitor.py and monitor_all.py: old versions of monitor_all.py used in real_monitor.launch)
	- (only real) audio_display.py: displays the audio signal and has some ideas for primative sound analysis
	- (only real) platform_control_server.py and control_test.py: ideas and test for using services instead of publisher/subscriber (topics)
	- (only real) recognize_objects.py and tensorflow_ros/object_recognition_node: categorizes the toys
	- (robot=sim01|rob01) sensor_to_pos.py: publishes odometry and tf of MiRo (used to try maps + visualization in rviz)
	- (robot=sim01|rob01) recorded_sounds_random_lights.py: can play back all the prepacked sounds on MiRo's P1 and P2 and also has an random light generator
		- send `rostopic pub /soundP[12] std_msgs/Int8 1` to start a loop over all available P1 or P2 sound messages, with the bug at the end
		- send `rostopic pub /soundP3 std_msgs/Int8 <Number>` to play the sound `Number` on P3 (default Number between 1-5)
		- send `rostopic pub -r <hz> /lights String a` to let the robot randomly flash it's leds `<hz>` times per second
	- (robot=sim01|rob01 follow_type=mics|core), sound_follow_1_2.py: two options of following a sound source, one using the spatial interface the other using just the crosscorrelation between the two microphones (used in real_monitor.launch/monitor_all.py)

**Other Interesting software:**
- ORB_SLAM2_ROS
- ratslam_ros (did not work with Miro?)
- RASL-MIRO (add closing parenthesis in flags.py, make mic_test.py easier, commented out import of sound interface in sound_test.py)
- miroDetection (changes in miroDetectorROS.py to make it work, e.g. add rospy.spin())
- Miro_SocialRobot (no changes)
- MiRo-follow.paralel.line.in.motion (no changes)
- Miro (no changes)
- ActiveHearing_onMiro (only change IP address in client.py)
- Kate etc.
- my other (first) scripts?

