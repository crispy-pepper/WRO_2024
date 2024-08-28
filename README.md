# SJJ | Explorer Robotics | WRO - FIRST Robitics | Canada

## Team Members <br>
- Sunni Xue
- Jayden Li
- John Weng
  **add team photo here**
## Content
<table>
<tr><th>Content of Repository</th><th>Content of README.md</th></tr>
<tr><td>
 
| Folder  | Content| 
| -- | -- |
| [`Team Photos`](/Team%20Photos)| 30 |
| [`Vehicle Photos`](/Vehicle%20Photos) | 301 |
| [`models`](/models) | 301 |
| [`others`](/others) | 301 |
| [`schemes`](/schemes) | 301 |
| [`scr`](/scr) | 301 |
| [`videos`](/videos) | 301 |

</td><td>
 
Section  | Content 
--- | --- |
[`Task`](#task) | 30 
[`Engineering Materials`](#engineering-materials) | 301 
[`Our Approach`](#our-approach) | 301 
[`Assembly Instructions`](#assembly-instructions) | 301 

</td></tr> </table>

## Task 
![plot](/other/images-used/task.jpg)
Build a self-driving, autonomous vehicle that completes 2 challenges: the open challenge and the obstacle challenge. <br>
### Open Challenge
The open challenge is where the car must complete three full laps around the field. The size of each side of the field is determined by random chance of either 100 cm or 60 cm. The direction in which the car drives is also randomized. <br><br>
 
### Obstacle Challenge
The obstacle challenge is where the car must complete three full laps around the field, avoiding different coloured pillars. If the pillar is red, traverse on the right side; if the pillar is green, traverse on the left. The direction in which the car drives is randomized. After the third lap, depending on the last pillar, the car must continue or change directions to find the parking lot. The car must then back into the parking lot without touching the ends. The size of each side of the field remains constant, 1 metre for each side. <br><br>


## Engineering materials
**photo of vehicle**
### Car Base
* [Carisma 80468 GT24RS 1/24 4WD On-Road Brushless RTR Retro Rally Car as the structure, and steering system of the car](https://www.ebay.ca/itm/134622499234)<br>
<img src="/other/images-used/engineeringmaterials_carisma.jpg" height="150"><br>
* [Furitek Micro Komodo 1212 3450KV Brushless Motor](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118)<br>
<img src="/other/images-used/engineeringmaterials_motor.jpg" height="150"><br>
* [Furiteck Lizard Pro 30A/50A Brushless ESC](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth)<br>
<img src="/other/images-used/engineeringmaterials_ESC.jpg" height="150"><br>
* [Hitec HS-5055MG Servo](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y)<br>
<img src="/other/images-used/engineeringmaterials_servo.jpg" height="150"><br>
* [Gens Ace 2S1P 1300mAh 7.4V battery](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r)<br>
<img src="/other/images-used/engineeringmaterials_battery.jpg" height="150"><br>
* Rasberry PI 4 Model B)<br>
<img src="/other/images-used/engineeringmaterials_pi.jpg" height="150"><br>
* [Raspberry Pi 5 Expansion Board Model A(DC Port)](https://www.hiwonder.com/collections/expansion-board/products/expansion-board-for-raspberry-pi-5)<br>
<img src="/other/images-used/engineeringmaterials_pihat1.jpg" height="150"><img src="/other/images-used/engineeringmaterials_pihat2.png" height="150"><br>
* Mini Rocker Switch)<br>
<img src="/other/images-used/engineeringmaterials_switch.jpg" height="150"><br>
* 5V Mini Fan)<br>
<img src="/other/images-used/engineeringmaterials_fan.jpg" height="150"><br>
* [3d printed base](/models)
### Open Challenge
* [SainSmart Camera Module RPi3, 5MP, Fish-Eye](https://www.sainsmart.com/products/noir-wide-angle-fov160-5-megapixel-camera-module))<br>
<img src="/other/images-used/engineeringmaterials_camera.jpg" height="150"><br>
### Obstacle Challenge
* [SainSmart Camera Module RPi3, 5MP, Fish-Eye](https://www.sainsmart.com/products/noir-wide-angle-fov160-5-megapixel-camera-module)
* [BerryIMU V3 10DOF](https://www.amazon.com/BerryIMUv2-10DOF-Accelerometer-Gyroscope-Magnetometer-Barometric/dp/B072MN8ZRC))<br>
<img src="/other/images-used/engineeringmaterials_gyro.jpg" height="150"><br>
* Raspberry Pi ultrasonic sensor)<br>
<img src="/other/images-used/engineeringmaterials_ultrasonic.jpg" height="150"><br><br><br>

## Our Approach
### Software
**Open Challenge**: Our approach to this challenge was to detect the walls, turn when one wall disappears, and then count the number of turns to know when to end. <br><br>
 
**Obstacle Challenge**: Our approach to this challenge was to detect the pillars, adjust according to pillar colour, turn at the orange/blue lines, count the number of turns to know when the laps end, detect the parking lot, and back in using additional sensors. <br>

### Obstacle Management
**ss of cv2 window**
#### Wall Following/Track Centering
To make sure that laps stayed consistent and the vehicle did not touch the walls, we had to implement some form of track centering. We did this using a [SainSmart Camera Module RPi3, 5MP, Fish-Eye](#engineering-materials). With the mounted camera, we were able to capture the surroundings of the vehicle frame by frame. Using these captures, we applied four (left top, left bottom, right top, right bottom) unique ROIs (regions of interest) that encapsulated the walls diagonally ahead on both sides. We then created a black threshold mask to calculate how much area of the ROIs was black. Using these areas, we could determine if the vehicle is veering too far to one side by calculating the difference between the two sides.
<br><br>
To physically put this calculation into action, we used a Proportional-Integral-Derivative (PID) algorithm approach, although we only used proportional and derivative.
This algorithm calculates the precise angle the servo should turn by taking the difference between the two sides multiplied it by the proportional value (constant) and adding it to the derivative value (constant) multiplied by the difference between the current and last difference in the two sides. The use of PID control allows stable turning with less oscillating and overcorrection, ensuring that the vehicle remains centered on the track.
<br><br>
#### Turning
Similar to how we centered the vehicle, we also used the area of black in the ROIs to decide when to turn. If one ROI's black area was less than a certain value, it would mean that the wall has disappeared and the servo would turn to the most extreme angle. The vehicle would keep turning until the wall appeared again.
<br><br>
However, this did not always work because of the varying widths of each corner. To fix this, we added another trigger for the turning sequence: the lines on the mat. Another small ROI was added to detect orange and blue contours. Depending on which of the colours detected first, it would know which direction to turn. Because the walls were unreliable for this turn, we used a timed turn for this, meaning it would continue turning for a certain period of time without worrying about the surroundings. This approach also helped prevent issues with overturning and underturning at narrower corners.
<br><br>
#### Pillar Maneuvering: Obstacle Challenge Only
The camera scans for pillars using another ROI and a red and green mask. We would know the closest pillar by finding the largest contour. Depending on the colour of this contour, we could decide whether to go left or right. However, this posed many challenges with overturning, underturning, turning past before it got to the pillar, and not turning at all. We fixed this by adding a constant target value for both coloured pillars and adjusting according to the distance between the pillar's left x-value and the target line. The vehicle would constantly try to match the x-value up with the target line. This way, the vehicle would know to continue turning towards the pillar or to turn the other way to correct the overturning.
<br>

#### Backtracking: Obstacle Challenge Only
Because there was a limitation to how many degrees our vehicle could turn at a time, there was an issue of not turning enough in time. To solve this we would check how big the current pillar/wall was and calculate if the vehicle would make it past successfully (without touching or moving anything). If the vehicle could not, it would backtrack at the opposite angle, readjust and continue forwards. This would continue until the vehicle could successfully make it past.
<br>

#### 3-Point Turn: Obstacle Challenge Only
iseic
<br>
#### Parallel Parking: Obstacle Challenge Only
uydsc
<br><br><br>

### Hardware

#### Movement Considerations
* Servo for steering
* Motor for FWD (four-wheel drive)
* The vehicle [Chassis is a Carisma 80468 GT24RS 1/24](#engineering-materials) so the vehicle is small enough to fit in the parallel parking space vertically
* The components are mounted on a 3d printed base sitting on top of the chassis, with the motor and servo being mounted directly into the chassis
* The [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials) was chosen combined with a [Furiteck Lizard Pro 30A/50A Brushless ESC](#engineering-materials) because brushless motors are mechanically driven, which allows more precise speed controls, longer life and higher efficiency with less maintenance.
* The servo was chosen because.. (use engineering principles: speed, torque, power etc) <br>
<img src="/other/images-used/engineeringmaterials_motor.jpg" height="300"><img src="/other/images-used/engineeringmaterials_ESC.jpg" height="300">
<img src="/other/images-used/engineeringmaterials_servo.jpg" height="300"><br>


In both the open and obstacle challenge, vehicle movement is essential for ensuring optimal performance. The vehicle is managed through a four-wheel drive configuration, with front-wheel steering. This configuration resembles everyday cars on the street, and allows for movement forwards and backwards, as well as turning in both directions. <br>

For the propulsion of the vehicle, we choose the [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials) due to the high speeds that the motor can achieve, indicated by the 3450KV rating. High speeds allow rapid movements and give us headroom without needing to max out the motor while running. Brushless motors are known for providing consistent torque which allows the vehicle to move smoothly. Brushless motors are electrically driven, which allows more precise speed control, and have a longer lifespan with less maintenance, which is ideal for our vehicle. We paired the motor with a [Furiteck Lizard Pro 30A/50A Brushless ESC](#engineering-materials), which regulates the power delivered to the motor, allowing for smooth acceleration and deceleration and ensures the safety and consistency of the motor.<br>

We chose the [Hitec HS-5055MG Servo](#engineering-materials) for our steering capabilities. It is perfect for small vehicles, with high torque at 1.2kg/cm at 4.8V or 1.5kg/cm at 6V. This allows the turning capabilities of the car to be consistent due to the strength of the servo and ensures the vehicle can turn quickly and accurately at low speeds or when stationary, which is essential for this competition. <br>

The vehicle is based on the [Charisma 80468 GT24RS 1/24 model](#engineering-materials), which was selected due to its compact size, which allows the vehicle to parallel park vertically, making the job much easier. The small size also contributes to the overall agility of the vehicle, making it easier to maneuver in tight spaces. <br>

The motor and servo replace the original components in the [Charisma 80468 GT24RS 1/24 model](#engineering-materials), with the servo needing modifications to the chassis of the car. The other components of the car are attached through a 3d printed base that is clipped in on top of the car. 
(something something add the building/assembly instructions, CAD files and parts) <br>

#### Power Considerations
* Sensors and power management
* Reference schematic <br>
<img src="/other/images-used/engineeringmaterials_battery.jpg" width="500">

The power and sensor systems are critical to the vehicle's performance in navigating the challenges of the competition. For this project, the vehicle is powered by a [Gens Ace 2S1P 1300mAh 7.4V battery](#engineering-materials). This battery has a discharge rating of 45C, meaning it can provide up to 58.5 Amps of current, which is more than enough to meet the power requirement of the vehicle's components.

The battery serves as the power source for the vehicle, providing energy for all parts. The [Furitek Lizard Pro 30A/50A Brushless ESC](#engineering-materials) is connected to the battery to regulate the power delivered to the [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials). This ESC is required to ensure smooth acceleration and deceleration, protecting the motor from potential damage due to power surges.

The Raspberry Pi, which acts as the vehicle's brain, has a power consumption of 1280mA under heavy stress, shown by studies. Given the battery's capabilities, this power draw is much less than what the battery can provide, allowing the Raspberry Pi to operate properly while managing all sensor inputs and processing.

The vehicle's sensing capabilities include a [SainSmart Camera Module RPi3](#engineering-materials), which is used in both the open and obstacle challenges. The camera is equipped with a fish-eye lens, offering a wider field of view. This visual range enables the vehicle to detect obstacles, such as walls and pillars, and to more effectively process the surroundings. 

In the obstacle challenge, the camera detects the walls, the colour of the pillars, the parking space, and the lines on the course. The vehicle uses this information to change the direction left or right accordingly, ensuring it stays on course. The fish-eye lens enhances the vehicle’s ability to capture more of the field at closer distances, providing the system with more information to make timely decisions.

(also if we use ultrasonic sensor or BerryIMU ill add it later)<br>


<br>

## Assembly Instructions
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
*explain how to upload control to the raspberry pi

## Content

- `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
- `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
- `video` contains the video.md file with the link to a video where driving demonstration exists
- `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
- `src` contains code of control software for all components which were programmed to participate in the competition
- `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
- `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
