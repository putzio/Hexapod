# Hexapod

This is a project of a remotely controlled walking robot
If you click the link bellow you can see the movie, which presents the robot 
https://photos.app.goo.gl/Wpt8i5VjzHMQxFu78

## The project consists of 4 main parts: 

### Mechanical construction
I have designed the robot in Fusin 360, which allows to generate a link to the CAD model:

https://a360.co/3f0cuEE

### Electronics
The .sch and .brd files are located in the documentation directory

### Mobile App
to control the robot I have created a simple application:

https://gallery.appinventor.mit.edu/?galleryid=f348ef10-de80-403b-b9b3-069c0fd0a535

### C++ code for the raspberry pi pico

the main file is hexapod.cpp. It uses classes Servo, Leg and Body, which are included in the lib directory

If you woul like to build it on your own you can use VSC and configure your computer as it is shown in the link below:
https://www.youtube.com/watch?v=B5rQSoOmR5w

## Bachelor thesis 
I have described this project in my bachelor thesis and you can read it in the documentation directory (unfortunately it's in polish)  
##  Unity simulation
I have also created a simulation in Unity (using an older version of my mechanical construction), so you can check it out in my other repository:
https://github.com/putzio/Hexapod_Unity
