# SJJ | Explorer Robotics | WRO - FIRST Robitics | Canada

## Team Members <br>
- Sunni Xue
- Jayden Li
- John Weng
## Task 
Build a self-driving, autonomous vehicle that completes 2 challenges: the open challenge and the obstacle challenge. <br>

## Our Approach
### Car Body
### Open Challenge
The open challenge is where the car must complete three full laps around the field. The size of each side of the field is determined by random chance of either 100 cm or 60 cm. The direction in which the car drives is also randomized. <br>
Our approach to this challenge was to detect the walls, turn when one wall disappears, and count the number of turns to know when to end. <br><br>

**Track Centering:** To check whether our car was in the middle of the track, we took live camera captures of the field in front of the vehicle. Using four regions of interest (ROI), two on each side, we would detect the size of the black walls and compare them against each other. If one side was significantly larger than the other, it meant the car was too skewed towards that side. To actually center the vehicle, we used a Proportional, Integral and Derivative (PID) wall follower approach by calculating the difference between the two walls. This would help mitigate the chances of overcorrection.

**Turning:** Similar to track centering, we used four ROIs to detect the size of the black walls. If one wall was completely gone, we would know to turn towards that side. Because we experienced issues with detecting the entire front wall and detecting the next wall too early, we set it so that once the turn had started, it would continue to turn until a certain period of time was up. This would remove any instances of premature stopping, overturning, underturning, and overcounting turns.
 
### Obstacle Challenge
The obstacle challenge is where the car must complete three full laps around the field, avoiding different coloured pillars. If the pillar is red, traverse on the right side; if the pillar is green, traverse on the left. The direction in which the car drives is randomized. After the third lap, depending on the last pillar, the car must continue or change directions to find the parking lot. The car must then back into the parking lot without touching the ends. The size of each side of the field remains constant, 1 metre for each side. <br>
Our approach to this challenge was to detect the pillars, adjust according to pillar colour, turn at the orange/blue lines, count the number of turns to know when the laps end, detect the parking lot, and back in using additional sensors. <br><br>

## Engineering materials
### Car Base
* [Carisma 80468 GT24RS 1/24 4WD On-Road Brushless RTR Retro Rally Car as the structure, and steering system of the car](https://www.ebay.ca/itm/134622499234)
* Komodo Motor
* [Hitec HS-5055MG Servo](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y)
* Furitek Lizard Pro ESC
* [Gens Ace 2S1P 1300mAh 7.4V battery](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r)
* Rasberry PI 4 Model B
* Raspberry Pi Multi-function Expansion Board 
* Rasberry PI fan
* 3d printed base
### Open Challenge
* SainSmart Camera Module RPi3, 5MP, Fish-Eye
### Obstacle Challenge
* SainSmart Camera Module RPi3, 5MP, Fish-Eye
* [BerryIMU v3](https://www.amazon.com/BerryIMUv2-10DOF-Accelerometer-Gyroscope-Magnetometer-Barometric/dp/B072MN8ZRC)
* Raspberry Pi ultrasonic sensor


## Content

- `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
- `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
- `video` contains the video.md file with the link to a video where driving demonstration exists
- `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
- `src` contains code of control software for all components which were programmed to participate in the competition
- `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
- `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
