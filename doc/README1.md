<<<<<<< HEAD
Link:
https://www.youtube.com/watch?v=LisY5gKp_Sg

Current state:
We have working in one file the detection of the square images,
and movement to a random location given a map file.
Also a launch file to start everything up except the python code to do
the movement and detection.

Not working:
currently that is all we have and we intend to code what is needed to do
the project.
The map doesnt show up on rviz.

To test our code assuming basic ros knowledge.
launch turtlebot.launch in the launch folder.
then after rziv camera section finishes loading.
run the python file in the scripts folder named detect_alvar.py.

then wait till the script finishes which should be after three movements.

The terminal will have lots of spam if it detects anything
(those are the messages it recieves).

the next checkpoint we want to make it so the robot stops its "random" movement
when it detects something then move to what it detected.
=======
Demonstration video: https://www.youtube.com/watch?v=LisY5gKp_Sg&

The functionality of our current code is the following:
 - The robot finds a random valid point on the map. It will either navigate there(SUCCESS) or 
	if it cannot find a path, it will abort navigation(ABORT). Once either state is reached, 
	it will find a new point until it has processed three random points.
 - While the program is active, the robot will detect alvar markers and print the data from 
	the messages using loginfo. 


What doesn't work:
 - We need to configure Rviz so that the map displays in Rviz and the user can see where the 
	robot thinks it is. Currently, the robot appears to be acting as though it is following the 
	map, but there is no visual feedback to confirm.


To run our code:
1. Launch "turtlebot.launch" in the launch directory. (roslaunch zeta_rescue turtlebot.launch)
2. Run "detect_alvar.py" in the scripts directory. (python detect_alvar.py)


Future development:

Some goals for checkpoint 2:
 - Integrate the Victim message in our implementation 
 - Come up with a search pattern that will work on any map but still find the most victims
 - Make the robot get in position and save a photo if it recognizes a victim.
The universal search pattern will probably be the largest task to take on, so it may not be implemented fully by then. 
However, we should be able to at least recognize and report the Victims that have alvar markers on them.
>>>>>>> cc5526016a63323624074cd4e713c1bd2db80a73
