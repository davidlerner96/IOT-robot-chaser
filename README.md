# IOT-robot-chaser
236333 - iot project: robot chaser

#So far: 

Unit tests: 
servo - input (via serial monitor) an angle and check if it turns correctly 
l298n (back TT motors) - input (via serial monitor) speed and direction for each motor and check if it is correct  

Robot's part: 
Robot tries to connect to a nearby wifi that is stored in memory. If there aren't any, then it creates an AP "ESP32_AP", creates an http page with a wifi manager interface that allows the user to enter wifi credentials, view hardware, and update firmware.  
After entering the credentials and hitting "save", the credentials are stored in the esp32's NVS and then the esp attempts to connect to it.  
The robot then creates an http server, that is accepting command for rear tt motors to drive at a certain speed their direction: 'B' or 'F'.  
And also a for the servo motor angle which is responsible for the steering or the front wheels. 

Lab's part: 

1. Implemented a script for receiving the robot's and the target's data (coordinates and rotation) from the Motive/Optitrack system, converting the angles representation from quaternions to euler angles and also converting
the initial axis system to Z-up, to match the initial orientation of the robot and target. 

2. Implemented a script to send http commands to the robot. 

3. Implemented the pure pursuit algorithm for tracking a moving target with smart mechanisms for a better and smoother tracking with a simulation to visualize and work on the theoretical logic of the robot's movement. 

#Edge cases: 

Reconnection attempts to wifi whenever connection is lost. As long as the attempts fail the led is blinking and the robot doesn't move. 

#Problems: 


#TODO: 

Main goal right now is to combine 1,2,3 scripts from Lab's part above so that simulation in 2 will be replaced by the actual data from the optitrack system in script 1, and the direction of motion will be calculated by the algorithm, then using script 3 to send the actual command for the robot to move. 

Then watch the actual motion result of the robot and make adjustment or add mechanisms accordingly.
