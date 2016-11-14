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
