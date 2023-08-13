# WRO-FE-2023

# Engineering Documentation | Team Krobotics | Canada
----

This repository contains engineering materials of The Crobot, a vehicle's model from Team Krobotics, for the WRO Future Engineers competition for the 2023 season.

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
others - other required photos

----



# Program arrangement and Algorithm Planning
The robot runs on a Raspberry Pi board, we also use an Arducam for Raspberry Pi Camera, OV5647 Pi Camera Module. It detects the surrounding environment including walls and red/green obstacles. The motor and steering servo are both connected to our Arduino Uno board. To count the number of laps, we see how many times the robot is in the “turning state.” Once it reaches 12, we stop the program(set speed to zero and angle to forward).

When the robot is turned on, it will communicate with the Raspberry Pi, and then waits for the code to run. The servo motor is set to center and the motor speed is set to zero.


**Step 1 - Starting Run:**

We start by accelerating the motor, and begin in the wall-following state. We draw contours and calculate the area of the wall on either side. The difference between the areas is proportional to the distance from the car to the center of the lane.

**Step 2 - Avoiding Walls:**

We run a PID(proportional, integral, derivative) controller to complete the run and avoid the walls. The robot tries to stay in the center of the track by adjusting its position based on the areas of the walls. If the left area is larger, it will correct the error by turning to the right and vice versa. Once the walls’ area on one side is almost nothing, we turn towards that side because it means that it is open and thus we turn in that direction.

**Step 3 - Avoiding Obstacles**

In order to turn correctly for the traffic lights, we have a region of interest at the front of the car which can detect the blocks. We then use HSV values to only capture the blocks, and we find the x and y coordinates of the pillar. Then we adjust to the left and right according to the colour of the pillar, relative to the x-coordinate of the pillar. The y-coordinate of the pillar will dictate how steep of a turn will be needed in order to get to the desired x coordinate, which will in turn mean going around the pillar.

**Step 4 - Counting Laps**

As previously mentioned, the car counts the number of laps by the number of times the car goes into its turning phase. Once it counts that it has turned 12 times, it means that 12 laps have occurred. Then, the program will stop.

**Mechanical Parts**
#Traxxas Trx4M
The Traxxas Trx4m was initially chosen due to the fact that it would be easy to modify, as well as being a great all around car which is able to handle all sorts of terrain, electrical components being compatible with ours, as well as fitting the size and weight restraints of the car.

#Raspberry Pi 4 Model B

The Raspberry Pi was chosen as it is a common computing device which allows for all sorts of coding and intricate use which will be helpful in the competition. Some examples will be communication with the arduino, attaching a camera and powersource, and using the vast amount of libraries that the pi has to offer.

#Arduino Uno R3

The Arduino was chosen for its popularity and for its great synergy with the Raspberry Pi. Also, it allowed for easy control of the DC motor and the Servo motor, with an easy to understand programming interface and great debugging.

#Furitek Micro Komodo Motor

The Furitek motor was used over the original Traxxas motor due to the fact that it was brushless versus the latter which was brushed. There are many benefits for using a brushless motor as opposed to a brushed motor, however to be brief it is just better.


#Traxxas LiPo Battery and charger

The battery and charger which powers the DC and Servo motor. The only changes made were connections with the wiring.


#Raspberry Pi Sugar 4

The Raspberry Pi Sugar was used to power the Pi, which also ended up powering the Arduino. This would be important in ensuring that both the Pi and Arduino would have sufficient power to run at its full capacity. However, a slight issue occurred as we have two power sources, and the restraints made it such that we could only have one switch to turn both of them on. More will be discussed about this later in the DPST switch section.

#DPST Switch

Due to the rules, it restricts us to having only one switch to fully turn on the robot. Due to the fact that we have both a power source for the Pi and Arduino, and a power source for the motors, it meant that these two connections had to be connected into one switch. This was done using the DPST switch.

#Mechanical Design

Using the Traxxas as a base, the chassis was expanded on using 3d printing elements and zipties in order to ensure the stability of the robot. Using cases and bases to hold the Pi and it’s respective power source was helpful in condensing the robot in size. Lego was also used during the prototyping stage.


#3D Parts

A base was designed in order to hold the Raspberry Pi, Arduino, wiring and some other pieces. A camera stand was designed to ensure that the camera would stay in the same place and be stable while driving




