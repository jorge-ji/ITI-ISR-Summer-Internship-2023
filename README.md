# ITI-ISR-Summer-Internship-2023

Controls the robotic arm Kinova Gen3 to perform a collaborative task with two other people. Uses an external camera to communicate the human actions to the robot.

Uses ROS Kortex and OpenPose.

To run the project, open a terminal and source the setup.bash in the catkin workspace where ROS Kortex is intalled. Run the script with the following command:
```
python build_tower.py <mode> (1 or 2) <config1> <config2> <config3> <config4> ... (can be any quantity)
```
- mode 1 is for condition 1: first 2 configurations are done with a robot that performs gaze actions, in the others, the robot lacks awareness.
- mode 2 is for condition 2: first 2 configurations are done with a robot that lacks awareness, in the others, the robot performs gaze actions.
- the configuration file look like the following (see the example files):
  - turn order (y (yellow) is the robot's, p (pink) is the left person's, b (blue) is the right person's)
  - object order (1 = square, 2 = rectangle, 3 = semicircle, 4 = bridge)
  - block 1 start position
  - block 1 target position
  - block 2 start position
  - block 2 target position
  - block 3 start position
  - block 3 target position<br />
Example:
```
python build_tower.py 1 config1 config2 config3 config4
```

Then, open another terminal and run the OpenPose script:
```
python openpose_server.py
```
The file has to be inside the built OpenPose folder (ex: openpose/build/examples/tutorial_python_api).
