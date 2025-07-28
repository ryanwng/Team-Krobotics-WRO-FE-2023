# WRO-FE-2023

# Engineering Documentation | Team Krobotics | Canada
----

This repository contains engineering materials of The Krobot, a vehicle's model from Team Krobotics, for the WRO Future Engineers competition for the 2023 season.

The theme "Connecting the World" focused on exploring how robots can improve shipping logistics and digial infrastuructre to promote safety, sustainability and efficiency. In the Future Engineers category, the competition was to design a self-driving autonomous robot vehicle which was capable of navigating around a randomly generated track with obstacles acting as "traffic lights."

----

## Team Members:

- Kevin Yang - email: kevin.yang.toronto@gmail.com
- Ryan Wang - email: wangryan833@gmail.com

----

## Repository Overview

models - contains 3D printable files for some parts of our robot

schemes - contains a schematic of the the electrical systems

src - contains the code for our robot

t-photos - contains team photos, one serious and one funny

v-photos - contains vehicle photos, from all directions

video - contains youtube links for videos of our robot

----



# Program arrangement and Algorithm Planning
The robot runs on a Raspberry Pi board, we also use an Arducam for Raspberry Pi Camera, OV5647 Pi Camera Module. It detects the surrounding environment including walls and red/green obstacles. The motor and steering servo are both connected to our Arduino Uno board. To count the number of laps, we see how many times the robot detects a low area of ROI one either side (which indicates when it turns). Once it reaches 12, we stop the program after a set amount of time.

When the robot is turned on, it will communicate with the Raspberry Pi, and then waits for a button to be pressed. Once the button is pressed, the servo motor is set to center and the motor speed is set to zero.


**Step 1 - Starting Run:**

We start by accelerating the motor, and begin in the wall-following state. We draw contours and calculate the area of the wall on either side. The difference between the areas is proportional to the distance from the car to the center of the lane.

**Step 2 - Avoiding Walls:**

We run a PID(proportional, integral, derivative) controller to complete the run and avoid the walls. The robot tries to stay in the center of the track by adjusting its position based on the areas of the walls. If the left area is larger, it will correct the error by turning to the right and vice versa. Once the walls’ area on one side is almost nothing, we turn towards that side because it means that it is open and thus we turn in that direction.

**Step 3 - Avoiding Obstacles**

In order to turn correctly for the traffic lights, we have a region of interest at the front of the car which can detect the blocks. We then use HSV values to only capture the blocks, and we find the x and y coordinates of the pillar. Then we adjust to the left and right according to the colour of the pillar, relative to the x-coordinate of the pillar. The y-coordinate of the pillar will dictate how steep of a turn will be needed in order to get to the desired x coordinate, which will in turn mean going around the pillar.

**Step 4 - Counting Laps**

As previously mentioned, the car counts the number of laps by the number of times the car detects lack of area in either left or right ROI. Once it counts that it has turned 12 times, it means that 3 laps have occurred. Then, the program will stop after 8 seconds to ensure it ends in the final area.

----



# Mechanical Parts

**Traxxas Trx4M**

The Traxxas Trx4m was initially chosen due to the fact that it would be easy to modify, as well as being a great all around car which is able to handle all sorts of terrain, electrical components being compatible with ours, as well as fitting the size and weight restraints of the car.

**Raspberry Pi 4 Model B**

The Raspberry Pi was chosen as it is a common computing device which allows for all sorts of coding and intricate use which will be helpful in the competition. Some examples will be communication with the arduino, attaching a camera and powersource, and using the vast amount of libraries that the pi has to offer.

**Arduino Uno R3**

The Arduino was chosen for its popularity and for its great synergy with the Raspberry Pi. Also, it allowed for easy control of the DC motor and the Servo motor, with an easy to understand programming interface and great debugging.

**Furitek Micro Komodo Motor**

The Furitek motor was used over the original Traxxas motor due to the fact that it was brushless versus the latter which was brushed. There are many benefits for using a brushless motor as opposed to a brushed motor. Brushless motors require less power, produce higher speed, and also make less noise.

**Traxxas LiPo Battery and charger**

The battery and charger which powers the DC and Servo motor. The only changes made were connections with the wiring.


**PiSugar S Pro**

The Raspberry Pi Sugar was used to power the Pi, which also ended up powering the Arduino. This would be important in ensuring that both the Pi and Arduino would have sufficient power to run at its full capacity. However, a slight issue occurred as we have two power sources, and the restraints made it such that we could only have one switch to turn both of them on. More will be discussed about this later in the DPST switch section.

**DPST Switch**

Due to the rules, it restricts us to having only one switch to fully turn on the robot. Due to the fact that we have both a power source for the Pi and Arduino, and a power source for the motors, it meant that these two connections had to be connected into one switch. This was done using the DPST switch.

**Mechanical Design**

We decided to use a Traxxas Trx4M as our base, the chassis was expanded on using 3D printed elements and zipties in order to ensure the stability of our parts. For example, our camera is stabilized on a 3D printed mount, which is stabilized through the use of glue and zipties. Our motor is a Furitek Micro Komodo Motor, it has 3 wires that connect to our esc to control speed and torque. This motor is directly compatible with the car we picked. Using cases and bases to hold the Raspberry Pi and its respective power source was helpful in condensing the robot in size. Lego was also used to create a camera stand to ensure that the camera would face down. Finally, tape was used in some areas, along with hot glue to increase the stability of the car in weaker areas.


**3D Parts**

A base was designed in order to hold the Raspberry Pi, Arduino, wiring and some other pieces. A camera stand was previously designed to ensure that the camera would stay in the same place and be stable while driving. However, it was figured later on that the camera wouldn't be able to face down, and so the stand was made with lego and some wooden pieces to ensure it would stay in place. 

**Hardware Issues**

There ended up being some hardware issues which he had to fix throughout the preparation of the competition. Firstly, the use of the button on the Pi Sugar S Pro was necessary in order to fulfill the requirements of the competition of using only one button to start the program. This meant adding additional code, along with solidfying the reason for using the Pi Sugar S Pro compared to other power sources, such as lithium batteries. Also, for the Raspberry Pi it would be necessary to code a way to run a program immediately upon turning on. This was done using the command "sudo crontab -e" which allows for programs to run upon rebooting of the raspberry pi. This alongside with the GPIO button on the Pi Sugar S pro was invaluable in allowing for a suitable car which can perform all its necessary functions. 


