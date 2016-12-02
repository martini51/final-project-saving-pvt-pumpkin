Demonstration video: 

The functionality of our current code is the following: 
  - all functionality from Checkpoint 1 
  - The program now takes and saves images when the alvar marker is sensed (using the Kinect).
      - Right now several images are taken if the robot is marker is within ~2 or 3 feet from the connect becuase we determined that was the furthest from the connect while maintaining image quality.  
  - Victim message is recorded and sent to victim topic using the correct information.
  - Fixed from last checkpoint, rviz now correctly shows the map.

What doesn't work:
  - We need to change the way navigating and robot wandering so it efficiently navigates the "disaster zone".
     - To complete this task we need to save the points and visited points(in between) and traversed that. 
  - Use the Logitech webcam to take images instead of the kinect which is too fuzzy to pick up the words on the victim's nametags. 
  - The optional description of the victim in Victim.msg. 


To run our code (Unchanged from checkpoint 1):
1. Launch "turtlebot.launch" in the launch directory. (roslaunch zeta_rescue turtlebot.launch)
2. Run "detect_alvar.py" in the scripts directory. (python detect_alvar.py)

goals for checkpoint 3: 
  - Take images with the Logitech webcam provided and save those images (instead of the kinect images) 
  - Figure out a correct way to mount the camera.
  - Fix movement to be better
  - Move back to start point
  - Movement should move to infront of victims and take clear photos
  - Take in command line arguments
  - Add announcment that it is done
  - Add orange circle to rviz where victim is found
