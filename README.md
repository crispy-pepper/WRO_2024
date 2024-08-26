# SJJ | Explorer Robotics | WRO - FIRST Robitics | Canada

## Team Members <br>
- Sunni Xue
- Jayden Li
- John Weng
## Task 
Build a self-driving, autonomous vehicle that completes 2 challenges: the open challenge and the obstacle challenge. <br>
## Engineering materials
### Car Base
* [Carisma 80468 GT24RS 1/24 4WD On-Road Brushless RTR Retro Rally Car as the structure, and steering system of the car](https://www.ebay.ca/itm/134622499234)
* [Furitek Micro Komodo 1212 3450KV Brushless Motor](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118)
* [Furiteck Lizard Pro 30A/50A Brushless ESC](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth)
* [Hitec HS-5055MG Servo](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y)
* [Gens Ace 2S1P 1300mAh 7.4V battery](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r)
* Rasberry PI 4 Model B
* Custom Raspberry Pi Multi-function Expansion Board
* Rocker Switch
* 5V Mini Fan
* [3d printed base](/models)
### Open Challenge
* [SainSmart Camera Module RPi3, 5MP, Fish-Eye](https://www.sainsmart.com/products/noir-wide-angle-fov160-5-megapixel-camera-module)
### Obstacle Challenge
* [SainSmart Camera Module RPi3, 5MP, Fish-Eye](https://www.sainsmart.com/products/noir-wide-angle-fov160-5-megapixel-camera-module)
* [BerryIMU V3 10DOF](https://www.amazon.com/BerryIMUv2-10DOF-Accelerometer-Gyroscope-Magnetometer-Barometric/dp/B072MN8ZRC)
* Raspberry Pi ultrasonic sensor<br><br>

## Our Approach
### Software
#### Open Challenge
The open challenge is where the car must complete three full laps around the field. The size of each side of the field is determined by random chance of either 100 cm or 60 cm. The direction in which the car drives is also randomized. <br>
Our approach to this challenge was to detect the walls, turn when one wall disappears, and then count the number of turns to know when to end. <br><br>

**Track Centering:** To check whether our car was in the middle of the track, we took live camera captures of the field in front of the vehicle. Using four regions of interest (ROI), two on each side, we would detect the size of the black walls and compare them against each other. If one side was significantly larger than the other, it meant the car was too skewed towards that side. To actually center the vehicle, we used a Proportional, Integral and Derivative (PID) wall follower approach by calculating the difference between the two walls. This would help mitigate the chances of overcorrection.

**Turning:** Similar to track centering, we used four ROIs to detect the size of the black walls. If one wall was completely gone, we would know to turn towards that side. Because we experienced issues with detecting the entire front wall and detecting the next wall too early, we set it so that once the turn had started, it would continue to turn until a certain period of time was up. This would remove any instances of premature stopping, overturning, underturning, and overcounting turns.

**Pillars** To  navigate around pillars, we implemented a color detection system using the camera's region of interest (ROI). This  ensures that the system accurately identifies the pillars while minimizing interference from background noise. Based on the detected color of the pillars, the vehicle adjusts itself, swerving left or right as needed. In cases where the vehicle detects that it is passing a pillar on the incorrect side, it automatically uses a reverse maneuver to realign itself and correct its course, ensuring compliance with the intended path.
 
### Obstacle Challenge
The obstacle challenge is where the car must complete three full laps around the field, avoiding different coloured pillars. If the pillar is red, traverse on the right side; if the pillar is green, traverse on the left. The direction in which the car drives is randomized. After the third lap, depending on the last pillar, the car must continue or change directions to find the parking lot. The car must then back into the parking lot without touching the ends. The size of each side of the field remains constant, 1 metre for each side. <br>
Our approach to this challenge was to detect the pillars, adjust according to pillar colour, turn at the orange/blue lines, count the number of turns to know when the laps end, detect the parking lot, and back in using additional sensors. <br>

### Hardware

#### Movement Considerations
* Servo for steering
* Motor for FWD (four-wheel drive)
* The vehicle [Chassis is a Carisma 80468 GT24RS 1/24](#engineering-materials) so the vehicle is small enough to fit in the parallel parking space vertically
* The components are mounted on a 3d printed base sitting on top of the chassis, with the motor and servo being mounted directly into the chassis
* The [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials) was chosen combined with a [Furiteck Lizard Pro 30A/50A Brushless ESC](#engineering-materials) because brushless motors are mechanically driven, which allows more precise speed controls, longer life and higher efficiency with less maintenance.
* The servo was chosen because.. (use engineering principles: speed, torque, power etc) <br>

In both the open and obstacle challenge, vehicle movement is essential for ensuring optimal performance. The vehicle is managed through a four-wheel drive configuration, with front-wheel steering. This configuration resembles everyday cars on the street, and allows for movement forwards and backwards, as well as turning in both directions. <br>

For the propulsion of the vehicle, we choose the [Furitek Micro Komodo 1212 3450KV Brushless Motor](#engineering-materials) due to the high speeds that the motor can achieve, indicated by the 3450KV rating. High speeds allow rapid movements and give us headroom without needing to max out the motor while running. Brushless motors are known for providing consistent torque which allows the vehicle to move smoothly. Brushless motors are electrically driven, which allows more precise speed control, and have a longer lifespan with less maintenance, which is ideal for our vehicle. We paired the motor with a [Furiteck Lizard Pro 30A/50A Brushless ESC](#engineering-materials), which regulates the power delivered to the motor, allowing for smooth acceleration and deceleration and ensures the safety and consistency of the motor.<br>

We chose the [Hitec HS-5055MG Servo](#engineering-materials) for our steering capabilities. It is perfect for small vehicles, with high torque at 1.2kg/cm at 4.8V or 1.5kg/cm at 6V. This allows the turning capabilities of the car to be consistent due to the strength of the servo and ensures the vehicle can turn quickly and accurately at low speeds or when stationary, which is essential for this competition. <br>

The vehicle is based on the [Charisma 80468 GT24RS 1/24 model](#engineering-materials), which was selected due to its compact size, which allows the vehicle to parallel park vertically, making the job much easier. The small size also contributes to the overall agility of the vehicle, making it easier to maneuver in tight spaces. <br>

The motor and servo replace the original components in the [Charisma 80468 GT24RS 1/24 model](#engineering-materials), with the servo needing modifications to the chassis of the car. The other components of the car are attached through a 3d printed base that is clipped in on top of the car. 
(something something add the building/assembly instructions, CAD files and parts) <br>

#### Power Considerations
* Sensors and power management
* Reference schematic <br>

The vehicle is powered by one [Gens Ace 2S1P 1300mAh 7.4V battery](#engineering-materials), which has 1300mAh with a discharge rating of 45C, which means it is capable of providing 58.5 Amps, which is more than capable of powering our car.<br>

The open and obstacle challenges both use a [SainSmart Camera Module RPi3](#engineering-materials) to detect and avoid walls, as well as for turning. The open challenge also uses the camera to detect pillars and colours. The camera has a fish-eye lens, which allows it to see more of the field from closer, which allows us to make decisions faster than a normal lens would allow. This gives us more information and time to make a decision. This camera captures colour in enough <br>

Add senor power consumption
(also if we use ultrasonic sensor or BerryIMU ill add it later)<br>


<br><br>


## Assembly Instructions
1. Disassembling the Chassis:
 - Begin by carefully unscrewing the cover of the Carisma 80468 GT24RS 1/24 chassis. This includes removing the top pole that supports the rear of the cover.
 - Next, unscrew the top shell of the vehicle. After the shell is removed, detach the components securing the servo and motor in place.

2. Creating Space for New Components:
 - Remove the battery holder from the chassis, followed by the front partition of the servo holder. This step is crucial to create sufficient space for the 
installation of the upgraded motor and servo.

3. Installing the New Motor:
 - Replace the original motor with the Furitek Micro Komodo 1212 3450KV Brushless Motor. Carefully position the motor within the chassis and secure it in place, 
ensuring that it aligns correctly with the existing drivetrain components.

4. Installing the New Servo:
 - Replace the original servo with the Hitec HS-5055MG Servo. Secure the servo by screwing a long screw into its end to properly engage the steering mechanism. 
 - Make sure to clip the standoffs into place for stability.

5. Wiring the Power System:
 - Connect the Gens Ace 2S1P 1300mAh 7.4V battery cable to the Furiteck Lizard Pro 30A/50A Brushless ESC cable. Follow the wiring configuration provided in the README of the schemes folder to ensure proper power delivery to the motor and other components.

6. 3D Printed Components Installation:
 - Print one “base v3.stl” and one “camera holder v6.stl” using a 3D printer.
 - Install the camera holder into the base by aligning it with the inset negatives on the base.
 - Mount the assembled base onto the poles that originally supported the vehicle’s cover, ensuring the camera holder is positioned over the rear wheels. Use the original pins from the car to securely clip the base in place.

7. Installing the Camera:
 - Secure the camera into the holder using screws, aligning it with the pre-drilled holes in the camera holder. Ensure the camera is firmly fixed to prevent any movement during operation.

8. Mounting the Battery:
 - Place the battery into the designated cutout within the camera holder's supports. Use tape, Velcro, or zip ties to secure the battery in place, ensuring it remains stable during vehicle operation.

9. Installing the Raspberry Pi:
 - Place the Raspberry Pi on top of the base, ensuring that the pins on the base align with the screw holes on the Raspberry Pi. Secure the Raspberry Pi in place using appropriate screws.

10. Wiring the Vehicle:
 - Carefully wire the car together according to the provided schematic, ensuring all connections are secure and correctly aligned. Pay special attention to wire management to prevent tangling or obstruction of moving parts.

11. Final Checks:
 - Ensure all components are securely mounted and that there is no excess movement. Double-check all wiring connections for correctness and stability.

12. Running the Code:
 - Once the assembly is complete, upload the control code to the Raspberry Pi. Ensure all systems are functioning correctly by performing a series of preliminary tests, such as motor and servo response, camera feed, and sensor readings.


## Content

- `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
- `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
- `video` contains the video.md file with the link to a video where driving demonstration exists
- `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
- `src` contains code of control software for all components which were programmed to participate in the competition
- `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
- `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
