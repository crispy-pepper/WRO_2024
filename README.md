# SJJ | WRO - Future Engineers | Canada

## Team Members 
- Sunni Xue
- Jayden Li
- John Weng
  **add team photo here**
## Content 📖📚
 
| Folder  | Content| 
| -- | -- |
| [`Team Photos`](/Team%20Photos)| Informal team photo, formal team photo |
| [`Vehicle Photos`](/Vehicle%20Photos) | [Bottom view](/Vehicle%20Photos/bottom_view.jpg), [front view](/Vehicle%20Photos/front_view.jpg), [left view](/Vehicle%20Photos/left_view.jpg), [rear view](/Vehicle%20Photos/rear_view.jpg), [right view](/Vehicle%20Photos/right_view.jpg), [top view](/Vehicle%20Photos/top_view.jpg) |
| [`models`](/models) | [Final base model](/models/base%20v3.stl), [final camera base model](/models/camera%20base.stl), [final camera holder model](/models/camera%20holder%20v5.stl), [expansion board diagram](/models/expansion_board_diagram.png), [final fan holder model](/models/fan%20holder%20v2.stl), [old and unused models (zipped)](/models/old) |
| [`other`](/other) | [Images used in documentation](other/images-used) |
| [`schemes`](/schemes) | [Schematic explanations](/scheme/README.md), [expansion board schematic](/schemes/Raspberry%20Pi%20Expansion%20Board%20Schematic.png), [Raspberry Pi schematic](/schemes/Raspberry%20Pi%20Schematic.png), [vehicle schematic](/schemes/Vehicle%20Schematic.png) |
| [`src`](/src) | Obstacle challenge final, [open challenge final](/src/OpenChallengeFinal.py), [HSV finder](/src/HSVRange.py), [test files](/src/Tests) |
| [`videos`](/videos) | Open challenge, obstacle challenge final |

 
 
Section  | Content 
--- | --- |
[`Task`](#task) | Introduces the problem/task of this competition
[`Engineering Materials`](#engineering-materials) | Engineering materials used to complete the vehicle 
[`Obstacle Management`](#obstacle-management) | Obstacle Management
[`Assembly Instructions`](#assembly-instructions) | How to build the vehicle


## Task
![plot](/other/images-used/task.jpg)
Obstacle challenge field

### Problem Statement
[The self-driving car challenge in this season is a Time Attack race: there will not be multiple cars at the same time on the track. Instead, one car per attempt will try to achieve the best time by driving several laps fully autonomously. The traffic signs indicate the side of the lane the vehicle has to follow.](https://www.wro2022.org/theme/future-engineers)

In order to perform the race, the car must drive three laps. The vehicle is not allowed to move or knock down the traffic signs.

**GOAL: Build a self-driving, autonomous vehicle that completes 2 challenges: the open challenge and the obstacle challenge.** 
### Open Challenge
The open challenge is where the car must complete three full laps around the field without touching a wall. The size of each side of the field is determined by random chance of either 100 cm or 60 cm. The direction in which the car drives is also randomized. 
 
### Obstacle Challenge
The obstacle challenge is where the car must complete three full laps around the field, avoiding different coloured traffic signs(pillars). If the pillar is red, traverse on the right side; if the pillar is green, traverse on the left. The direction in which the car drives is randomized. After the third lap, depending on the last pillar, the car must continue in the same direction or change directions to find the parking lot. The car must then back into the parking lot without touching either of the ends. The size of each side of the field remains constant, 1 metre for each side. 


## Engineering materials
**photo of vehicle**
### Car Base
* [Carisma 80468 GT24RS 1/24 4WD On-Road Brushless RTR Retro Rally Car as the structure, and steering system of the car](https://www.ebay.ca/itm/134622499234)
<img src="/other/images-used/engineeringmaterials_carisma.jpg" height="150">
* [Furitek Micro Komodo 1212 3450KV Brushless Motor](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118)
<img src="/other/images-used/engineeringmaterials_motor.jpg" height="150">
* [Furiteck Lizard Pro 30A/50A Brushless ESC](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth)
<img src="/other/images-used/engineeringmaterials_ESC.jpg" height="150">
* [Hitec HS-5055MG Servo](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y)
<img src="/other/images-used/engineeringmaterials_servo.jpg" height="150">
* [Gens Ace 2S1P 1300mAh 7.4V battery](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r)
<img src="/other/images-used/engineeringmaterials_battery.jpg" height="150">
* Rasberry PI 4 Model B)
<img src="/other/images-used/engineeringmaterials_pi.jpg" height="150">
* [Raspberry Pi 5 Expansion Board Model A(DC Port)](https://www.hiwonder.com/collections/expansion-board/products/expansion-board-for-raspberry-pi-5)
<img src="/other/images-used/engineeringmaterials_pihat1.jpg" height="150"><img src="/other/images-used/engineeringmaterials_pihat2.png" height="150">
* Mini Rocker Switch)
<img src="/other/images-used/engineeringmaterials_switch.jpg" height="150">
* 5V Mini Fan)
<img src="/other/images-used/engineeringmaterials_fan.jpg" height="150">
* [3D printed base](/models)
* MicroSD card
### Sensors

* [SainSmart Camera Module RPi3, 5MP, Fish-Eye](https://www.sainsmart.com/products/noir-wide-angle-fov160-5-megapixel-camera-module)
<img src="/other/images-used/engineeringmaterials_camera.jpg" height="150">

## Obstacle Management
**ss of cv2 window**

<<<<<<< Updated upstream
- **Libraries Used** - `math` - `sys` - `cv2` - `numpy` - `time` - `picamera2` - `HiwonderSDK.Board` - `libcamera`

#### Wall-Following/Track Centering
=======
##### Wall Following/Track Centering
>>>>>>> Stashed changes

Our open and obstacle challenge used the same wall following algorithm that guaranteed the robot to remain in the center of the two walls when needed. To make sure that laps stayed consistent and the vehicle did not touch the walls, we had to implement some form of track centering. We did this using a [SainSmart Camera Module RPi3, 5MP, Fish-Eye](#engineering-materials). With the mounted camera, we were able to capture the surroundings of the vehicle frame by frame. 

Using these captures, we applied four (left top, left bottom, right top, right bottom) unique ROIs (regions of interest) that captured the areas of the walls diagonally ahead on both sides. We then created a black threshold mask to calculate how much area of the ROIs was black. Using these areas, we could determine if the vehicle is veering too far to one side by calculating the difference between the two sides and adjust the robot accordingly. To physically implement this calculation, we used a Proportional-Derivative (PD) algorithm approach, deciding that a combining the two factors would be perfect for our goal of following the walls.


```py
error = left_area - right_area
turn (error)(proportional gain) + (change in error value over time)(derivative gain)
```


This algorithm calculates the precise angle the servo should turn by taking the difference between the two sides multiplied it by the proportional value (constant) and adding it to the derivative value (constant) multiplied by the difference between the current and last difference in the two sides. The use of PID control allows stable turning with less oscillating and overcorrection, ensuring that the vehicle remains centered on the track.

### Open Challenge

#### Turning

Our initial turning algorithm was simple in premise: when one of the walls was no longer detected, the robot would turn that direction. In practice, this algorithm performed inadequately because when turning into a narrow section, the robot would not turn at a great enough angle and therefore veer too close to the wall.


```py
if left_area is none
	turn sharp left until left_area is detected
else if right_area is none
	turn sharp right until right_area is detected
```


Realizing that waiting for the wall to be passed would not provide an adequate turn, our goal was to create a turn algorithm that could pre-emptively detect that a turn was needed. Our solution was to use the blue and orange lines on the mat to decide when to turn. Another small ROI was added to detect orange and blue contours. 

Depending on which of the colours was detected first, the algorithm would turn the correct direction accordingly. The turn would be ended after the opposite colour line was detected and the respective wall was detected again on the camera.

The same logic for if a wall was no longer detected from the first algorithm was also used in conjunction, creating an algorithm that would turn in a more optimized path. 


```py
if orange line detected
	turn sharp right until blue line detected
	
else if blue line detected
	turn sharp left until orange line detected
	
if left_area is none
	turn sharp left until left_area is detected
else if right_area is none
	turn sharp right until right_area is detected
```


This algorithm performed much better, but had a flaw: the line would be detected and the turn would start, but once the orange line was detected again, the turn would end. Therefore, we created a solution that would allow for the wall to fully be passed before ending the turn. This solution was adding a short timer to the turn that would ensure that the turn was not ended early.


```py
if orange line detected
	turn sharp right until blue line detected
	
	if blue line detected
		increment turn_timer
		
	if turn_timer greater than turn time
		end the turn
	
else if blue line detected
	turn sharp left until orange line detected
	
	if orange line detected
		increment turn_timer
	
	if turn_timer greater than turn time
		end the turn
		
if left_area is none
	turn sharp left until left_area is detected
	
else if right_area is none
	turn sharp right until right_area is detected
```


This algorithm was both reliable and efficient, allowing the robot to travel at high speeds with no risk of hitting the walls. 

### Obstacle Challenge

#### Turning
Similar to how we centered the vehicle, we also used the area of black in the ROIs to decide when to turn. If one ROI's black area was less than a certain value, it would mean that the wall has disappeared and the servo would turn to the most extreme angle. The vehicle would keep turning until the wall appeared again. This naïve approach was straightforward but failed in many cases because of the pillar-avoidance requirement:


```py
if left_area is none
	turn sharp left until left_area is detected
else if right_area is none
	turn sharp right until right_area is detected
```


However, this did not always work because of the varying widths of each corner. To fix this, we added another trigger for the turning sequence: the lines on the mat. This algorithm was very similar to the line detection in the open challenge without the timed aspect later added to that algorithm. 


```py
if orange line detected
	turn sharp right until blue line detected
	if right_area is detected
		stop turning
	
if blue line detected
	turn sharp left until orange line detected
	if left area is detected 
	stop turning
```


If a pillar was detected in the turn, the pillar-avoidance variables would be changed in order to enter the straight section while passing by the obstacle correctly. This would be achieved by making the y-axis proportional steering more sensitive. 

<<<<<<< Updated upstream
#### Pillar Maneuvering: 
The camera scans for pillars using another ROI that encapsulates the center of the camera view and a red and green colour mask. The algorithm would find the closest pillar by finding the largest contour. Depending on the colour of this contour, we could decide whether to go left or right. We started with a naïve approach of turning a constant amount left or right when the pillar was detected.
=======
##### Pillar Maneuvering: 
The camera scans for pillars using another ROI that encapsulates the center of the camera view and a red and green colour mask. The algorithm would find the closest pillar by finding the largest contour. Depending on the colour of this contour, we could decide whether to go left or right. We started with a naïve approach of turning a constant amount left or right when the pillar is detected.
>>>>>>> Stashed changes


```py
if red_area greater than pillar_threshold
	turn right
else if green_area greater than pillar_threshold
	turn left
```


However, this posed many challenges with overturning, underturning, turning past before it got to the pillar, and not turning at all. We fixed this by adding a constant target value for both coloured pillars and adjusting according to the distance between the pillar's left x-value and the target line. The vehicle would constantly try to match the x-value up with the target line. This way, the vehicle would know to continue turning towards the pillar or to turn the other way to correct the overturning. Additionally, we found that it would be beneficial for the robot to turn at a greater angle if the pillar is closer to avoid the pillar in urgent situations. Therefore, we added another factor into our turn degree: y-axis gain. This functionality would turn the servo motor at a greater angle based on the y-coordinate of the pillar, which is the straight distance forward from the robot. 


```py
if red_area greater than pillar_threshold
	error = target - red_pillar_x
	turn (error)(pillar proportional gain) 
	+ (change in error value over time)(pillar derivative gain)
	+ (y axis gain)(red_pillar_y)
	
```


<<<<<<< Updated upstream
#### Backtracking: 
Because there was a limitation to how many degrees our vehicle could turn at a time, there was an issue of not turning enough in time. To solve this, we would check how big the current pillar/wall was and calculate if the vehicle would make it past successfully (without touching or moving anything). If the vehicle could not, it would backtrack at the opposite angle, readjust, and continue forward. This would continue until the vehicle could successfully make it past.
=======
##### Backtracking: 
Because there was a limitation to how many degrees our vehicle could turn at a time, there was an issue of not turning enough in time. To solve this we would check how big the current pillar/wall was and calculate if the vehicle would make it past successfully (without touching or moving anything). If the vehicle could not, it would backtrack at the opposite angle, readjust and continue forwards. This would continue until the vehicle could successfully make it past.
>>>>>>> Stashed changes


```py
if pillar_area greater than avoidable distance and pillar_x is not on the correct side:
	reverse the robot
```


#### 3-Point Turn

The three point turn algorithm is required when the last pillar of the second lap is red. This signals that the robot must complete the final lap in the opposite direction. A three point turn is required to change the orientation of the robot. The algorithm is simple and effective. If the three point turn is needed, the robot will turn sharply right until a wall is seen. This allows for the robot to achieve the maximum angle from one turn. Once a wall is detected in front of the robot, the second phase of the turn will be activated and the servo will turn to a sharp left angle and reverse the robot. Finally, the robot turns at a sharp right angle and goes forward, ending in the opposite orientation from where it started. This allows the robot to complete the last lap in the appropriate direction.

```
three point turn
	turn sharp right 
	go forward
	if wall is directly in front of robot
		turn sharp left
		reverse the robot
		turn sharp right
		go forward
		end the three point turn
	repeat
		
if last pillar is red
	go past the pillar
	start three point turn
```

#### Parking
jayden will do

## Movement Considerations
* Servo for steering
* Motor for FWD (four-wheel drive)
* The vehicle [Chassis is a Carisma 80468 GT24RS 1/24](#engineering-materials) so the vehicle is small enough to fit in the parallel parking space vertically
* The components are mounted on a 3d printed base sitting on top of the chassis, with the motor and servo being mounted directly into the chassis
* The [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials) was chosen combined with a [Furiteck Lizard Pro 30A/50A Brushless ESC](#engineering-materials) because brushless motors are mechanically driven, which allows more precise speed controls, longer life and higher efficiency with less maintenance.
* The servo was chosen because.. (use engineering principles: speed, torque, power etc) <br>
<img src="/other/images-used/engineeringmaterials_motor.jpg" height="300"><img src="/other/images-used/engineeringmaterials_ESC.jpg" height="300"><img src="/other/images-used/engineeringmaterials_servo.jpg" height="300">


In both the open and obstacle challenge, vehicle movement is essential for ensuring optimal performance. The vehicle is managed through a four-wheel drive configuration, with front-wheel steering. This configuration resembles everyday cars on the street, and allows for movement forwards and backwards, as well as turning in both directions. 

For the propulsion of the vehicle, we choose the [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials) due to the high speeds that the motor can achieve, indicated by the 3450KV rating. High speeds allow rapid movements and give us headroom without needing to max out the motor while running. Brushless motors are known for providing consistent torque which allows the vehicle to move smoothly. Brushless motors are electrically driven, which allows more precise speed control, and have a longer lifespan with less maintenance, which is ideal for our vehicle. We paired the motor with a [Furiteck Lizard Pro 30A/50A Brushless ESC](#engineering-materials), which regulates the power delivered to the motor, allowing for smooth acceleration and deceleration and ensures the safety and consistency of the motor.

We chose the [Hitec HS-5055MG Servo](#engineering-materials) for our steering capabilities. It is perfect for small vehicles, with high torque at 1.2kg/cm at 4.8V or 1.5kg/cm at 6V. This allows the turning capabilities of the car to be consistent due to the strength of the servo and ensures the vehicle can turn quickly and accurately at low speeds or when stationary, which is essential for this competition. 

The vehicle is based on the [Charisma 80468 GT24RS 1/24 model](#engineering-materials), which was selected due to its compact size, which allows the vehicle to parallel park vertically, making the job much easier. The small size also contributes to the overall agility of the vehicle, making it easier to maneuver in tight spaces. 

The motor and servo replace the original components in the [Charisma 80468 GT24RS 1/24 model](#engineering-materials), with the servo needing modifications to the chassis of the car. The other components of the car are attached through a 3d printed base that is clipped in on top of the car. 
(something something add the building/assembly instructions, CAD files and parts) 

## Power Considerations
* Sensors and power management
* Reference schematic 
<img src="/other/images-used/engineeringmaterials_battery.jpg" width="500">

The power and sensor systems are critical to the vehicle's performance in navigating the challenges of the competition. For this project, the vehicle is powered by a [Gens Ace 2S1P 1300mAh 7.4V battery](#engineering-materials). This battery has a discharge rating of 45C, meaning it can provide up to 58.5 Amps of current, which is more than enough to meet the power requirement of the vehicle's components.

The battery serves as the power source for the vehicle, providing energy for all parts. The [Furitek Lizard Pro 30A/50A Brushless ESC](#engineering-materials) is connected to the battery to regulate the power delivered to the [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials). This ESC is required to ensure smooth acceleration and deceleration, protecting the motor from potential damage due to power surges.

The Raspberry Pi, which acts as the vehicle's brain, has a power consumption of 1280mA under heavy stress, shown by studies. Given the battery's capabilities, this power draw is much less than what the battery can provide, allowing the Raspberry Pi to operate properly while managing all sensor inputs and processing.

The vehicle's sensing capabilities include a [SainSmart Camera Module RPi3](#engineering-materials), which is used in both the open and obstacle challenges. The camera is equipped with a fish-eye lens, offering a wider field of view. This visual range enables the vehicle to detect obstacles, such as walls and pillars, and to more effectively process the surroundings. 

In the obstacle challenge, the camera detects the walls, the colour of the pillars, the parking space, and the lines on the course. The vehicle uses this information to change the direction left or right accordingly, ensuring it stays on course. The fish-eye lens enhances the vehicle’s ability to capture more of the field at closer distances, providing the system with more information to make timely decisions.

<br><br><br>

## Assembly Instructions
### Software
**1. Raspberry Pi OS**
 - Download and install the official Raspberry Pi Imager from [https://www.raspberrypi.com/software/]

**2. Turbo Pi**
 - Using Raspberry Pi Imager, copy the TurboPi operating system onto a microSD card
 - Insert into the Raspberry Pi
   
**3. Connecting to the Raspberry Pi**
   - Download and install RealVNC from [https://www.realvnc.com/en/connect/download/viewer/]
   - Plug in the Raspberry and wait until wifi access point “HW-xxxxxx” shows up in your wifi list
   - Launch RealVNC and create a new connection with server `192.168.149.1`
       - Username: pi
       - Password: raspberry
   - 
**3. Auto-running the program on start**
  - Transfer OpenChallengeFinal.py/ObstacleChallengeFinal.py onto the Raspberry Pi and open command prompt
  - Open command prompt and run `sudo nano /etc/rc.local`
  - Add the line `sudo bash -c 'sudo python3 /home/pi/<<directory>>/<<filename.py>>' &` before the line `exit 0` and save and close
  - Reboot the Raspberry Pi and the program should automatically run on start
   
<br><br>
### Hardware
**1. Disassembling the Chassis:**
 - Begin by unscrewing the cover of the Carisma 80468 GT24RS 1/24 chassis. This includes removing the top pole that supports the rear of the cover.
 - Next, unscrew the top shell of the vehicle. After the shell is removed, detach the components securing the servo and motor in place.

**2. Creating Space for New Components:**
 - Remove the battery holder from the chassis, followed by the front partition of the servo holder. This step creates sufficient space for the 
installation of the new motor and servo.

**3. Installing the New Motor:**
 - Replace the original motor with the Furitek Micro Komodo 1212 3450KV Brushless Motor, using the original motor compartment and holder.

**4. Installing the New Servo:**
 - Replace the original servo with the Hitec HS-5055MG Servo. Screw a long screw into the servo horn/arm so that it moves the steering mechanism. 
 - Make sure to clip the standoffs of the servo to fit into the chassis.

**5. Wiring the Power System:**
 - Connect the Gens Ace 2S1P 1300mAh 7.4V battery cable to the Furiteck Lizard Pro 30A/50A Brushless ESC cable. To ensure proper power delivery to the motor and other components, follow the wiring provided in the [README](/schemes/README.md) of the schemes folder

**6. 3D Printed Components Installation:**
 - Print one “base v3.stl” and one “camera holder v6.stl” using a 3D printer.
 - Install the camera holder into the base by aligning it with the inset negatives on the base.
 - Mount the assembled base onto the poles that originally supported the vehicle’s cover, ensuring the camera holder is positioned over the rear wheels. Use the original pins from the car to securely clip the base in place.

**7. Installing the Camera:**
 - Secure the camera into the holder using screws, aligning it with the holes in the camera holder. Ensure the camera is firmly fixed to prevent any movement during operation.

**8. Mounting the Battery:**
 - Place the battery into the  cutout within the camera holder's supports. Use tape, Velcro, or zip ties to secure the battery in place, ensuring it remains stable during vehicle operation.

**9. Installing the Raspberry Pi:**
 - Place the Raspberry Pi on top of the base, ensuring that the pins on the base align with the screw holes on the Raspberry Pi.

**10. Wiring the Vehicle:**
 - Wire the car together according to the provided schematic, ensuring all connections are secure and correctly aligned. Pay attention to wire management to prevent tangling or obstruction of moving parts or obstruction of the camera.

**11. Final Checks:**
 - Ensure all components are secure and that there is no excess movement. Double-check all wiring connections for correctness and stability.

**12. Running the Code:**
 - Once the assembly is complete, upload the control code to the Raspberry Pi. Check all systems are working by performing a series of tests that can be found in the tests folder located in the src folder.


## Content

- `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
- `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
- `video` contains the video.md file with the link to a video where driving demonstration exists
- `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
- `src` contains code of control software for all components which were programmed to participate in the competition
- `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
- `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
