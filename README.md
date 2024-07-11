# IOT-robot-chaser
236333 - iot project: robot chaser

#So far: 

Robot tries to connect to a nearby wifi that is stored in memory. If there aren't any, then it creates an AP "ESP32_AP", creates an http page with a wifi manager interface that allows the user to enter wifi credentials, view hardware, and update firmware. After entering the credentials and hitting "save", the credentials are stored in the esp32's NVS and then the esp attempts to connect to it. 
The robot then creates an http server, that is accepting command for rear tt motors to drive at a certain speed their direction: 'B' or 'F'. And also a for the servo motor angle which is responsible for the steering or the front wheels.

#Edge cases: 

Reconnection attempts to wifi whenever connection is lost. As long as the attempts fail the led is blinking and the robot doesn't move.

#Problems: 


#TODO: 

edit server "Get" command to control the servo motor for steering.
