# Other software and information

This file describes some of the other software I created or used. It also includes links to many interesting papers regarding the topic and additional information.

## List of links
Here is a list of useful links of libraries, papers, forum posts related to this project.

- Other project using MiRO:
	- [MiRo-follow.paralel.line.in.motioncapture](https://github.com/Enrichetto/MiRo-follow.paralel.line.in.motioncapture)
	- [Miro_SocialRobot](https://github.com/prajval10/Miro_SocialRobot)
	- [RASL-MIRO](https://github.com/partlygloudy/RASL-MIRO)
	- [Miro](https://github.com/MinhongW/Miro)
	- [ActiveHearing_onMiro](https://github.com/saeidmokaram/ActiveHearing_onMiro)
	- [miroDetection](https://github.com/mattdoubleu/miroDetection) and [dissertation](http://www.academia.edu/35067347/MSc_A_Vision-Based_Study_on_Collective_Behaviour_in_Mammal-like_Robots)
- Mapping:
	- [SLAM Explanation](https://husarion.com/tutorials/ros-tutorials/6-slam-navigation/)
	- [Twist to Odom](https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html)
	- Sonar
		- [Mapping with SONAR](https://github.com/RiccardoGiubilato/ros_autonomous_car)
		- [Mapping with Sonar/Faking LaserScan](https://answers.ros.org/question/41550/mapping-with-sonar-data/)
		- [Localization Paper](http://rossum.sourceforge.net/papers/Localization/PosPosterv4.pdf)
		- [Indoor Mapping Paper](https://marinerobotics.mit.edu/sites/default/files/Tardos02ijrr.pdf)
	- Vision:
		- [General Paper](http://www.robots.ox.ac.uk/~lav/Papers/davison_iccv2003/davison_iccv2003.pdf)
		- RatSLAM:
			- [OpenSLAM Docs](https://openslam-org.github.io/openratslam.html)
			- [Git OpenRatSLAM](https://github.com/davidmball/ratslam) [Git fork OpenCV3](https://github.com/sem23/ratslam)
			- [Paper OpenRatSLAM](https://pdfs.semanticscholar.org/ec5c/4a3a1fb7b84c76bf09e8ca0115b161e9472a.pdf)
			- [OpenRatSLAM BrowserGame](http://www.araa.asn.au/acra/acra2011/papers/pap158.pdf)
			- [BatSLAM](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3554696/)
		- ORB_SLAM2:
			- [git ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
			- [project website](http://webdiis.unizar.es/~raulmur/orbslam/)
			- [orb to grid fork](https://github.com/abhineet123/ORB_SLAM2/blob/master/2d-grid-mapping.pdf)
			- [orb to grid demo vid](https://www.youtube.com/watch?v=HoE22wMhuKA)
- [Camera Calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- Feature Recognition:
	- Face Detection:
		- [ROS package face_detection](http://wiki.ros.org/face_detection_tracking) [git](https://github.com/phil333/face_detection)
	- Emotions:
		- [OpenFace git](https://github.com/TadasBaltrusaitis/OpenFace) [paper](https://www.cl.cam.ac.uk/~tb346/pub/papers/wacv2016.pdf)
		- [ROS bindings1](https://github.com/interaction-lab/openface_ros) [ROS bindings2](https://github.com/asselbor/features_face)
		- [Facial Action Coding System Explanation](https://imotions.com/blog/facial-action-coding-system/)
	- Face Recognition:
		- [openface git](https://github.com/cmusatyalab/openface) [paper](http://reports-archive.adm.cs.cmu.edu/anon/anon/2016/CMU-CS-16-118.pdf)
		- [ROS bindings + other stuff git](https://github.com/tue-robotics/image_recognition)
			- [age+gener estimation](https://github.com/yu4u/age-gender-estimation)
	- Pose Estimation:
		- [openpose git](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
		- [openpose ROS wrapper](https://github.com/stevenjj/openpose_ros) [report](https://upcommons.upc.edu/bitstream/handle/2117/110929/ReportMArduengo6authors-1.pdf?sequence=3&isAllowed=y)
		- [tf-pose-estimation git](https://github.com/ildoonet/tf-pose-estimation)
		- [posenet git](https://github.com/tensorflow/tfjs-models/tree/master/posenet)
		- [posenet ROS wrapper](https://github.com/hansonrobotics/ros_posenet)
	- Sound:
		- [AUROS git](https://github.com/tyiannak/AUROS)
		- [ROAR paper](https://pdfs.semanticscholar.org/0b1e/ec864bb1dec63f0d1500a730f4427354d891.pdf)
		- [urban sound classification](http://aqibsaeed.github.io/2016-09-03-urban-sound-classification-part-1/)
		- [audio classifier tutorial](https://davidglavas.me/lets-build-an-audio-classifier/)
		- [comparison paper](https://www.sciencedirect.com/science/article/pii/S0167865503001478)
		- [use images to classify sound paper](https://www.sciencedirect.com/science/article/pii/S1877050917316599)
		- [librosa libary](http://librosa.github.io/librosa/index.html)
- Monitoring behaviour:
	- [effect on privacy paper](https://ieeexplore.ieee.org/document/6249578?part=undefined%7Cfig1)
	- [Home Bhevaior-Monitoring Robot paper](https://www.hindawi.com/journals/jhe/2017/6952695/)
	- [HUman Activity Recognition paper](http://www.ingentaconnect.com/content/asp/jmihi/2013/00000003/00000003/art00017;jsessionid=2ws7mst4p6xa7.x-ic-live-03)

## Own applications (in `other_scripts`)

#### client_testing *files: other_scripts/miro_ros_client_testing.py*
To run this program: `$ ./miro_ros_client_testing.py robot=<robot_name> [test=True]`.
Without the test option the robot will go straight until he recognizes an obstacle, turn approximatly 180deg, and cycle.
With the test option the robot will go to sleep, if it is bright, and wake up, if it is dark. Also, it should send some messages to core_config to do some noise and express some behavior, but this is not working correctly. Sometimes something happens, sometimes not.
**Note: the core behavior specified in core_config only works if you send control messages at at least 1hz to MiRo, not ALL messages will work, e.g. core_config messages won't work, the safest bet is to send empty core_control messages (if the drive is 0, they don't do anything).**

#### occupancy_grid *files: sensor_to_pos.py, other_scripts/range_to_occupancy_grid.py*
The idea is that Miro drives around (autonomously) and builds a occupancy grid map. The problem is that MiRo only has ONE sonar sensor, which means that you do not have enough information to build a "usable" map.
1. start roscore, bridge as normal
2. atm: autonomous drive is not available, use teleop or similar to drive him around
	- autonomous exploring will be implemented later on
	- or use *experimental* `$ ./explore.py robot=rob01`:
		- this will cause the robot to drive forward slowly turning its head and turning its body (90degs) if it is close to an obstacle 
		- problem, robot can get stuck easily
			- possible solutions: find out what the P1 and P2 messages mean (e.g. blinking green or red) and moving backward if stucked
3. run `$ ./sensor_to_pos.py` to publish a tf message (position of Miro (base_link) in respect to the world frame)
	- uses the optical shaft encoders of the wheels (twist.linear.x (mm/s), twist.angular.z (rad/s) from platform_sensors) to calculate a positon of the robot in the world (x,y,theta (m,m,rad)), when you start the program MiRo is at 0,0 with 0 rotation.
	- also changes the range data and publishes it 
	- also adds the sonar_link to the tf frame as a dynamic link (changing the yaw, will change the quaternation)
	- improve accuracy: x is kinda good, y has a higher error 
4. run `$ ./range_to_occupancy_grid.py` to publish a nav_msg/Occupancy_grid-msg at /miro/map
	- atm: somehow works
		- improve setting of obstacles (miro's movement is not too finegrained, therefore there are a some holes in the detected obstacles)
			- maybe fill in ?
		- when you close this program it saves the map as a csv (name default map.csv) (can easy be read and transformed to the grid again using numpy)
			- improvement save the map with a timestamp in the title
		- further work has to be done here!
5. run `$ rviz` to visualize Miro position and the map
	- add tf to see where MiRo is
	- add a map (linked to /miro/map with Occupancy_grid--msg)
6. An alternative is to only run `$ ./sensor_to_pos.py robot=rob01` and use gmapping instead of `range_to_occupancy_grid.py`. You can start gmapping with `rosrun gmapping slam_gmapping _scan:="scan" _odom_frame:="odom" _base_frame:="base_link" _delta:="0.1"` *Note: install with `sudo apt-get install ros-kinetic-gmapping`*. The problem is, that the results are not significantly better. 

#### SafeOperation Background Node *files: background_safety.py*
MiRo can move around very fast and has not too many sensors to sense danger in the environment. If you use the demo/autonmous behaviour and the robot is aroused, it could happen that the robot will damage itself. Therefore we had the idea that in the background a safety program should run permanently to protect the robot from damage. This program would stop all actions as soon as it detects a dangerous situation and then tries to solve it. This did not really worked and was dropped at an early stage of development, so treat the code as very experimental and the following is more about the original ideas and not about what it is actually doing.
> This node should always run in the background to ensure that no damage is dealt to the robot. Using the sonar and the camera and the signals from P1 (e.g. joints are blocked), the robot stops it's current behavior and retreats to a safe place. Publishes a topic `/safe` and all other tools (at least all that send move commands to the robot) should listen to this topic and set themselves to inactive if the `/safe` topic send a True? message. The node sends a True message to the `/safe` topic as soon as something dangerous is detected and resolves the problem, afterswards and in idle it send a False message to the topic. 
> - saves last body move (8 uiojklm,.) and last head move (X?) to come to an clever solution
> - also saves last P1 error codes
> - publish Int 8 message on topic_root/safe , 0 means safe everything else means a problem was detected 
> - every Node that sends control msgs to Miro should subscribe to this topic and set itself to active is safe is 0 and inactive el


## Other applications 

### Non-ROS

#### General tipps for Docker
Some general tipps regarding the use of docker. Actually, none of the ROS applications below use docker, but you can test some applications (e.g. OpenFace) using docker and a webcam, before you install the ROS wrappers.
- to change the default data directory: follow [this](https://forums.docker.com/t/how-do-i-change-the-docker-image-installation-directory/1169)
- to run GUI applications add the options `-e DISPLAY=$DISPLAY -v /temp/.X11-unix:/tmp/.X11-unix --net=host` [see here](http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/)
- to add a shared folder to the docker container (read and write) add the option `-v /path/to/folder/host:/path/in/docker`

#### OpenFace TadasBalturusatis [git](https://github.com/TadasBaltrusaitis/OpenFace)
A nice library to do *face detection/face landmark detection/head pose tracking/facial action unit recognition/gaze tracking*. See [here](https://www.cl.cam.ac.uk/%7Etb346/pub/papers/wacv2016.pdf) for a paper describing it. I use an old version of this library in features_face/OpenFace and miro_background/real_monitor.launch to recognize emotions.
If you want to test a more recent version with images/videos or a webcam, follow the instructions below.

- Install docker `$ sudo apt-get install docker-ce`
- Run `$ xhost +x local:docker`
- *Note: The first time you run this, it will download OpenFace* To run it with enabled GUI support and a shared sample folder run `$ docker run -it -e DISPLAY=$DISPLAY -v /temp/.X11-unix:/tmp/.X11-unix -v /path/to/samples:/samples --net=host algebr/openface:latest`
	- then (in docker) run `# ./build/bin/FaceLandmarkVid -f /samples/file.ending`
	- or use the webcam
		- add the option `--device=/dev/video0:/dev/video0` 
		- run `# ./build/bin/FeatureExtraction -device 0 -verbose`

#### openface cmusatyalab [git](https://github.com/cmusatyalab/openface)
A library to do face recognition using deep neural networks. Used in image_recognition/openface_ros.

- Install docker `$ sudo apt-get install docker-ce`
- Run `$ xhost +x local:docker`
- *Note: The first time you run this, it will download openface* To run the application run `docker run -p 9000:9000 -p 8000:8000 -t -i -e DISPLAY -v /temp/.X11-unix:/tmp/.X11-unix --net=host --device=/dev/video0:/dev/video0 bamos/openface /bin/bash` 
	- Webcam detection:
		- `# cd root/openface`
		- `# demos/classifer_webcam.py models/openface/celeb-classifier.nn4.small2.v1.pkl --captureDevice 0`
		- create an own classification model [tutorial](http://cmusatyalab.github.io/openface/demo-3-classifier/)
- To use the web application run `docker run -p 9000:9000 -p 8000:8000 -t -i bamos/openface /bin/bash -l -c '/root/openface/demos/web/start-servers.sh'`
	- find out docker ip address `$ docker inspect <docker_id> | grep "IPAddress"` (docker id via docker ps) (default: 172.17.0.2:8000)
	- then connect to `https://<docker-ip>:8000` in your browser

##### OpenPose [git](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
A library to perform multi-person keypoint detection for body, face and hands pose-estimation. This works, but if you don't have a cuda-enabled graphics card it will be very slow (~0.1fps with an i7-8550U). Instead, I used [posenet](https://github.com/tensorflow/tfjs-models/tree/master/posenet) in my final application, which was way faster (~15fps with an i7-8550U). You could also have a look at [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation).

- Follow the installation instructions [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md)
- `$cd <path_to_openpose>`
- e.g. Run `./build/examples/openpose/openpose.bin --image-dir <image-dir> --face --hand --write_images <output_dir>`
- To use the webcam run `./build/examples/openpose/openpose.bin --face --body_disable`

### ROS

#### miro_ros_client_gui *included in the MDK*
This is the main demo program included in the MDK by Consequential Robotics. It is good to get a feeling for MiRo. It displays most of MiRo's sensor data and allows you to send many basic commands (e.g. drive forward, blink, do pirate sounds) to him. Also, you can use it to start MiRo's demo-mode.
To start the program, run `$ ./miro_ros_client_gui.py robot=<robot_name>` where the file is saved. The default name of the robot is `sim01` for the simulation and `rob01` for the real robot. Now you see a lot of buttons and they do what is written on them. 

#### Kate (University of Aberystwyth) *ask for the files*
This is a program by Kate slightly modified by me (e.g. I added the functionality to quit the program by `ctrl+c`). It only works with the real robot and shows the functionality of many of the sensors of MiRo and lets you play some games with him. 
To start the program, run `$ ./technicalSubmission` where the file is saved. 
The program has many functionalities. At start the robot will react to various stimuli, e.g. when you touch his head or his body, he will also track red objects and react to light. Also, it includes two games, the first one is activated, when you show him a face, and will play some sound and flash a random color on his body, the second one, is activated by touching all 4 touch sensors in his body simultanously, and will play music and tries to chase an object (distance between the robot and the object stays the same). 

#### miroDetection [git](https://github.com/mattdoubleu/miroDetection)
*You have to make some minor changes to make the files runable, eg. missing parenthesis at the end of miroDetectorROS.py and to add rospy.spin()*
Can detect other Miros and human faces. Works with the camera stream from miro or with static images.
Uses ANN and Greyscale Histograms or SURF and Bayesian Estimation.
Run with `./miroDetectorROS.py robot=rob01`. Has a nice report included.
Problems: it is super slow (needs around 18sec for one image )

#### MiRo-follow.paralel.line.in.motioncapture [git](https://github.com/Enrichetto/MiRo-follow.paralel.line.in.motioncapture)
Should orient towards a goal and moves towards it, including obstacle avoidance.
Run `roslaunch straightmiro miro.launch`.
Problems: does not work?

#### Miro_SocialRobot [git](https://github.com/prajval10/Miro_SocialRobot)
Detects and classify touch pattern with MiRo. Doing deeplearning using TensorFlow(keras) and react differently to different patterns.
To run `./Data_input.py` and in another terminal `./miro_action.py`
Problems: false positives are high, but works 

#### RASL-MIRO [git](https://github.com/partlygloudy/RASL-MIRO)
*You have to make some small changes to make it runnable, e.g. outcommenting importing a sound_interface which is not existing and to add a closing parenthesis in flags.py*
A bunch of small test applications.
Not ROS-style, therefore run with `python pet_test.py|sound_test.py|brain.py|mic_test.py|eyelid_test.py`.
pet_test: react to pet (likes) and pat (dislikes)
sound_test: plays a sound (on P3)
mic_test: will approach the direction of the loudest sound source
brain.py: does some action, depending on its mood?
eyelid_test.py: no idea, not working?

#### Miro [git](https://github.com/MinhongW/Miro)
A small game, a test application for sound recognition and some image recognition (not really working)

#### ActiveHearing_onMiro [git](https://github.com/saeidmokaram/ActiveHearing_onMiro)
*You have to chaneg the IP address in client.py*
Some sound recognition, processing. 
Problems: not working out of the box?

#### OpenRatSLAM [git](https://github.com/sem23/ratslam) *sensor_to_pos.py*
Package to perform SLAM and create a topological map using only a monocular image stream. It did not work with MiRo, but I'm not sure why and believe it should work in principle. It works very well with different rosbag files they provide.
- First follow the installation instructions on github and download a rosbag file, e.g. iRat2011. Then you can launch it by running `$ roslaunch ratslam_ros irataus.launch` and `$ rosbag play irat_aus_28112011.bag`. 
- If you want to try it with MiRo, you have to provide odometry information, therefore run `$ ./sensor_to_pos.py robot=rob01`. You also have to redirect the camera stream to `/miro/camera/image`, e.g. by running `$ rosrun topic_tools relay /miro/rob01/caml/compressed /miro/camera/image/compressed`. You also have to change some parameters in the launch file, `$ rosed ratslam_ros config_irataus.txt.in`, most importantly the `topic_root` to `miro`. 
- If you want to create a rosbag file to do the mapping offline: run `$ rosbag record -O <name_of_bagfile> miro/camera/image miro/odom`. 

#### ORB_SLAM2 [git](https://github.com/raulmur/ORB_SLAM2)
Another package to perform SLAM using monocular, stereo or RGB-D image streams. It worked somehow with MiRo, but the quality of the map was too bad to be useful.  
- You have to remap MiRo's cameras to `/camera/image_raw`
	- Monocular version: `$ rosrun topic_tools relay /miro/rob01/platform/camr /camera/image_raw`
	- Stereo version: `$ rosrun topic_tools relay /miro/rob01/platform/camr /camera/right/image_raw &` and `$ rosrun topic_tools relay /miro/rob01/platform/caml /camera/left/image_raw &`
- Follow the installation instructions on github, `$ cd <path_to_ORB_SLAM2>` and run `$ rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml`
- Or for the stero version: run `$ rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoc.yaml true`.
- To get better results, you have to change the configuration parameters in the `yaml`-files. 
- The problem is, ORB_SLAM2 only displays the map and does not publish the information to ROS nor saves it, therefore use a fork, which can do this: e.g. [ORB2_to_grid](https://github.com/abhineet123/ORB_SLAM2)
	- follow the installation instructions there
	- to run `rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml -1 /miro/rob01/platform/caml`
	- then `rosrun ORB_SLAM2 Monosub < scale_factor > < resize_factor > < cloud_max_x > < cloud_min_x > < cloud_max_z > < cloud_min_z > < free_thresh > < occupied_thresh > < use_local_counters > < visit_thresh > < use_gaussian_counters > < use_boundary_detection > < use_height_thresholding > < normal_thresh_deg > < canny_thresh > < enable_goal_publishing > < show_camera_location > < gauss_kernel_size >` 
	- Parameters used in the orginal paper: 30 1 10 -10 22 -12 0.45 0.40 1 10 1 1 1 30 400
	- now you can see the map in rviz (published to /grid_map) and eventually save and use it
	- TODO: calibrate correctly, find suitable parameters, build an environment  

#### Spatial Interface *included in MDK*
This is not an application, but an interface you can use to interact with MiRo. The official documentation is very crude, therefore I added some extra documentation.
You have to send the flag SPATIAL_ENABLE to the core/config to activate the spatial behavior.
You will now have an audio event and the priority peaks of three attentional priority maps in /core/state. If you activate the additional flag SPATIAL_SEND_PRIORITY you can look at the priority maps at /core/[pril|prir|priw] and if you send the flag SPATIAL_SEND_OTHER you can look at the move map and the rgb map at /core/[movl|movr] and core/[rgbl|rgbr]. 
- first two priority maps (work somehow):
	- first *camera left*, second *camera right*
	- azim: where the event is angulary located referenced to MiRo's head in radian. 1.5 ~ 90 deg to the left, -1.5 ~ 90 deg to the right
	- elev: where the event is heightly located. near 0 (or 0.2) bottom of MiRo's camera, near 1 (or 1.5) top of MiRo's camera
	- height: how large the peak is in the priority map, (new moving object becomes 1.0 then decreases up to 0.0 if the image is perfectly still for a long time)
	- size: how much of the map is filled with that event (seems to be not really working)
	- t_complete: time stamp of when the map was calculated the last time
- last priority map:
	- aural (mircophone) map
	- azim: same
	- elev: always around 0.2 (probably does not work)
	- height: how important (loud) the vent is
	- size: does not really makes sense?
	- t_complete: same
- audio_event:
	- flags: most of the time 1 sometimes another number, found no documentation
	- sample_number: number of the sample in which the audio event occured
	- azim: angular location in radian from MiRo's head, only goes up to 90 and then down to 0 again 
	- elev: always around 0.2 (does not work?)
	- magnitude: how loud the event was between 0.0 and 1.0 (event has to be louder than a specific threshold to even occur) (but a additional threshold on it 0.1 for own applications)
	- t_complete: time of the audio event
- You can also publish the maps and look at the images directly, but this does not look fruitful for me