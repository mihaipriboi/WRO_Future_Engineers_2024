<center><h1> Nerdvana Cancer 2024 </center>

## Table of Contents
* [Photos](#team-image)
  * [Team](#team-image)
  * [Robot](#robot-image)
* [Performance Video](#video)
* [Mobility Management](#mobility-management)
  * [Powertrain](#powertrain-mechanical)
    * [Drivetrain](#drivetrain-mechanical)
    * [Motor](#motor-mechanical)
    * [Motor Driver](#motor-driver-mechanical)
  * [Steering](#steering-mechanical)
    * [Servo Motor](#servo-motor)
  * [Chassis](#chassis-mechanical)
* [Power and Sense Management](#power-and-sense-management)
  * [Li-Po Battery](#li-po-battery)
  * [Arduino Nano ESP32](#arduino-nano-esp32)
  * [IMU](#imu-sensor)
  * [OpenMV Cam H7 R2](#openmv-cam-h7-r2)
  * [Voltage Regulator](#voltage-regulator-(L7805CV))
  * [Circuit Diagram](#circuit-diagram)
* [Code for each component](#code-for-each-component)
  * [Drive Motor](#drive-motor-code)
  * [Servo Motor](#servo-motor-code)
  * [Camera](#camera-code)
  * [IMU](#gyro-sensor-code)
* [Obstacle Management](#obstacle-management)
  * [Qualification Round](#quali-management)
  * [Final Round](#final-management)
* [Randomizer](#randomizer)
* [Resources](#resources)
  * [3D Models](#3d-models-resources)
  * [Images](#images-resources)

### Team: Priboi Mihai, Nicola Victor, Bălan Teodor <a class="anchor" id="team-image"></a>
  ![Team](./images/team_image.jpg)

## Photos of our robot <b>TBD<b> <a class="anchor" id="robot-image"></a>

| <img src="./images/robot_images/robot_front.png" width="90%" /> | <img src="./images/robot_images/robot_back.png" width="85%" /> | 
| :--: | :--: | 
| *Front* | *Back* |
| <img src="./images/robot_images/robot_left.png" width="90%" /> | <img src="./images/robot_images/robot_right.png" width="85%" /> | 
| *Left* | *Right* |
| <img src="./images/robot_images/robot_top.png" width="90%" /> | <img src="./images/robot_images/robot_bottom.png" width="85%" /> | 
| *Top* | *Bottom* |

<br>

## Our video of the robot on [Youtube](https://youtu.be/) <a class="anchor" id="video"></a>

<br>

# Mobility Management <a class="anchor" id="mobility-management"></a>
![Powertrain](./images/resources/powertrain.png "Powertrain")

## Powertrain <a class="anchor" id="powertrain-mechanical"></a>

![Powertrain - Bottom View](./images/resources/powertrain_bottom_text.png "Powertrain - Bottom View")

### Drivetrain <a class="anchor" id="drivetrain-mechanical"></a>

To minimize friction and thereby reduce speed loss, we avoided using 3D-printed components for the moving parts in the drivetrain. Instead, we utilized Lego pieces, which are molded with high precision and therefore have a very low friction coefficient. The 3D-printed parts were reserved for the chassis and for mounting the electronic components onto the Lego structure.

### Motor <a class="anchor" id="motor-mechanical"></a>

Following an evaluation of different motors, we settled on a geared DC motor that comes with a magnetic encoder. This motor was selected for its lightweight and compact design, which stands out among others with comparable output. Additionally, the magnetic encoder offers greater precision than its optical counterpart. We secured the motor to the chassis using screws.

**Specifications:**
- Voltage: 12V
- Gear Ratio: 1:50
- Speed: 650 ± 31% rpm
- Torque: 0.67 ± kg·cm
- Weight: 9.5g

Where to buy the motor: https://www.pololu.com/product/3039

To connect the motor's axle to a Lego-compatible axle, we created a custom 3D-printed adapter.

![Gearmotor to axle - 3D Model](./images/resources/DriveMotorToLegoAxle.jpg "Gearmotor to axle 3D piece")

### Motor Driver <a class="anchor" id="motor-driver-mechanical"></a>
![Motor driver](./images/resources/motor_driver.png "Motor driver")

To control the speed of the drive motor, we utilized a SparkFun Dual TB6612FNG motor driver.

Where to buy the motor driver: https://www.sparkfun.com/products/14450

## Steering <a class="anchor" id="steering-mechanical"></a>

![Powertrain - Angled Bottom View](./images/resources/bottom_angle.png "Powertrain - Angled Bottom View")

After experimenting with various steering mechanisms such as Ackermann steering and bell-crank steering, we assessed their advantages and drawbacks. Ultimately, we chose a straightforward steering system consisting of a parallelogram linkage. This decision was made because the alternative systems were either too large or too complex to implement effectively. Our selected mechanism is simple, light, and compact, providing a satisfactory steering angle. While it does not adhere to the Ackermann steering geometry, our tests showed that for our robot's small size and light weight, this omission was not critically significant.

### Servo Motor <a class="anchor" id="servo-motor"></a>
![MG90S Servo](./images/resources/MG996R.webp "MG90S Servo")

For steering, we selected the MG90S servo motor, favoring it for its high torque and swift response.

**Specifications:**
- Weight: 13.4g
- Stall torque: 2.2 kgf·cm (6V)
- Operating speed: 0.08 s/60 degree (6V)
- Rotation angle: 120 degree

Where to buy the servo motor: https://www.sigmanortec.ro/servomotor-mg996r-180-13kg

To connect the servo motor to the steering system, we fashioned a custom 3D-printed adapter. Given the dynamic geometry of the system, the connector couldn't be a single rigid piece because its length needed to be adjustable according to the wheel positions. Thus, we designed a two-part beam: the larger piece attaches to the servo and the smaller piece to the steering mechanism. The smaller piece slides into the larger one, permitting the beam's length to vary. We introduced a slight space between the two components to ensure smooth movement, while also ensuring that a significant portion of the smaller piece remains within the larger one to avoid disconnection or bending.

![Servo Arm - 3D Model](./images/resources/ServoArm.png "Servo Arm")

## Chassis <a class="anchor" id="chassis-mechanical"></a>

Our initial prototypes utilized Lego pieces to assemble all components, which simplified testing and design modifications. Once we finalized the design, we transitioned to a custom 3D-printed chassis for assembling the mechanical parts. This choice proved to be superior, resulting in a lighter and more compact structure that offered greater design flexibility compared to the Lego-based framework. The chassis accommodates the drivetrain and steering mechanism and includes designated mounts for securing the drive motor and servo motor.

![Chassis - 3D Model](./images/resources/Chassis.jpg "Chassis - 3D Model")


# Power and Sense Management <a class="anchor" id="power-and-sense-management"></a>

### Li-Po Battery <a class="anchor" id="li-po-battery"></a>
![Li-Po Battery](./images/resources/battery.png "Li-Po Battery")

Where to buy the battery: https://www.sierra.ro/Acumulator-LiPo-GENS-ACE-Soaring-7-4-V--2200-mA--20C-p11141p.html

The battery is mounted using a custom 3D-printed holder, which is secured to the chassis.

![Battery Mount - 3D Model](./images/resources/BatteryMount.jpg "Battery Mount - 3D Model")

### Arduino Nano ESP32 <a class="anchor" id="arduino-nano-esp32"></a>
![Arduino Nano ESP32](./images/resources/arduino.jpg " Arduino Nano ESP32")

At WRO2022 Future Engineers, we used a Raspberry Pi 3B+ in conjunction with an Arduino Uno Every through a serial connection. The Raspberry Pi was tasked with processing images from the camera, whereas the Arduino was dedicated to controlling the motors and collecting sensor data. This method, however, was not optimal. The processing speed of the Raspberry Pi fell short of our requirements, the serial link between the two boards was not as quick as necessary, and it also carried the risk of data loss.

Following the international competition in Dortmund, we discovered the Pixycam 2.1, a camera equipped with its own processing capabilities that can directly connect to an Arduino or other microcontrollers. With the Pixycam 2.1, we were able to remove the Raspberry Pi, thereby enhancing our robot's speed. We also upgraded from the Arduino Uno Every to a Teensy 4.1, which boasts much faster processing power.

Where to buy the Arduino Nano ESP32: https://store.arduino.cc/products/nano-esp32

The Arduino, mounted on a prototype board, is secured to the chassis with a custom 3D-printed holder.

![PCB Mount - 3D Model](./images/resources/PCB_Mount.jpg "PCB Mount - 3D Model")

### IMU <a class="anchor" id="imu-sensor"></a>
![IMU Sensor - BMI088](./images/resources/gyro.jpg "IMU Sensor - BMI088")

One importat aspect that helps the roboy navigate is the inertial measurement unit (IMU). After using the MPU6050 last year, we decided to upgrade to a better IMU. This sensor is based on BOSCH BMI088, which is a high-performance IMU with high vibration suppression. While the IMU measure the angular velocity and the acceleration of the robot, we only use the angular velocity to calculate the angle of the robot. The IMU is wired to the SDA and SCL pins on the teensy.

**Specifications:**
- Gyroscope range: ±2000°/s
- Accelerometer range: ±24g

Where to buy the gyro sensor: https://www.seeedstudio.com/Grove-6-Axis-Accelerometer-Gyroscope-BMI088.html

### OpenMV Cam H7 R2 <a class="anchor" id="openmv-cam-h7-r2"></a>
![OpenMV Cam H7 R2](./images/resources/pixy.png "OpenMV Cam H7 R2")

Where to buy the OpenMV Cam H7 R2: https://openmv.io/products/openmv-cam-h7-r2

The OpenMV Cam H7 R2 is mounted on the chassis with a custom 3D-printed holder.

![OpenMV Cam Mount - 3D Model](./images/resources/CameraMount.jpg "OpenMV Cam Mount - 3D Model")

### Voltage regulator <a class="anchor" id="voltage-regulator"></a>
![Voltage regulator (L7805CV)](./images/resources/linear_voltage_regulator.png "Voltage regulator (L7805CV)")

To provide the Arduino Nano ESP32 with the required 5V, we needed to decrease the output from the 7.4V battery, which can reach up to 8.4V when fully charged. We employed a linear voltage regulator, the L7805CV, capable of converting input voltages below 35V down to a steady 5V.

Where to buy the 5V voltage regulator: https://ro.mouser.com/ProductDetail/STMicroelectronics/L7805CV?qs=9NrABl3fj%2FqplZAHiYUxWg%3D%3D

### Circuit diagram <a class="anchor" id="circuit-diagram"></a>
![Circuit diagram](./electrical-diagram/circuit.png "Circuit diagram")

<br>

# Code for each component <a class="anchor" id="code-for-each-component"></a>

## Drive Motor <a class="anchor" id="drive-motor-code"></a>

## Servo Motor <a class="anchor" id="servo-motor-code"></a>

## Camera <a class="anchor" id="camera-code"></a>

## IMU <a class="anchor" id="gyro-sensor-code"></a>

To utilize the gyro sensor, we needed to include the _BMI088.h_ library. During initialization, we allocate a 10-second window to measure the sensor's drift, allowing us to refine the robot's angular readings for greater precision. Additionally, we configure the sensor's output data rate to 400Hz and set the bandwidth to 47Hz. The bandwidth determines the frequency of data sampling by the sensor; a higher bandwidth yields more precise data at the cost of increased power consumption. We also designate pin 15 as an input and attach an interrupt to it, enabling us to capture data from the sensor as soon as it becomes available.

Within the *read_gyro* function, we're retrieving data from the gyro sensor and adjusting it to account for any detected drift, enhancing the accuracy of the readings. Since the gyro provides data in radians, a conversion to degrees is necessary for our application. We're focusing solely on the rotation around the x-axis, hence we only compute the *gx* value, which represents the robot's angular rotation in degrees on that specific axis.

<br>

# Obstacle Management <a class="anchor" id="obstacle-management"></a>

## Qualification Round <a class="anchor" id="quali-management"></a>

For the qualifying round, we set up a basic switch-case system to guide our robot. This system tells the robot what to do next, depending on where it is. The robot knows where it is by counting how many times it has turned.

We use two main switch cases: *PID*, and *STOP*.

In the *PID* case, the robot just moves straight. It uses a special tool (PID controller) with a gyro sensor to stay on a straight line. If it gets too close to the wall, it gets a trigger from the camera to keep the right distance from the walls and make a turn.

## Final Round <a class="anchor" id="final-management"></a>

<br>

# Randomizer <a class="anchor" id="randomizer"></a>

To ensure the robot's ability to adapt to any course, we developed a randomizer that generates a random sequence of colors and positions for the cubes. You can find this web application at the following link: https://nerdvana.ro/wro-fe/

<br>

# Resources <a class="anchor" id="resources"></a>

## 3D Models <a class="anchor" id="3d-models-resources"></a>
<li> DC Motor - https://grabcad.com/library/
<li> MG90S Servo motor - https://grabcad.com/library/
<li> Sparkfun Motor Driver - https://grabcad.com/library/sparkfun-motor-driver-dual-tb6612fng-1a-1
<li> Arduino Nano ESP32 - https://grabcad.com/library/
<li> OpenMV Cam H7 R2 - https://grabcad.com/library/
<li> LiPo Battery - https://grabcad.com/library/2s-7-4v-li-po-battery-1
<li> Grove BMI088 Gyroscope - https://grabcad.com/library/mpu6050-1
<li> Linear Voltage Regulator - https://grabcad.com/library/linear-voltage-regulators-78xx-1
<li> Prototype Board - https://grabcad.com/library/pcb-board-prototype-3x7cm-1

<br>

## Images <a class="anchor" id="images-resources"></a>
<li> DC Motor - 
<li> Sparkfun Motor Driver - https://cdn.sparkfun.com//assets/parts/1/2/4/8/2/14450a-01.jpg
<li> MG90S Servo motor - 
<li> Arduino Nano ESP32 - 
<li> OpenMV Cam H7 R2 - 
<li> LiPo Battery - https://www.autorc.ro/16064-large_default/acumulator-lipo-gens-ace-3s-111v-2200mah-20c-mufa-xt60.jpg
<li> Grove BMI088 Gyroscope - https://files.seeedstudio.com/wiki/Grove-6-Axis_Accelerometer-Gyroscope-BMI088/img/main.jpg
<li> Linear voltage regulator - https://ro.farnell.com/productimages/standard/en_GB/GE3TO220-40.jpg

<br>

## Copyright <a class="anchor" id="copyright"></a>

Unless explicitly stated otherwise, all rights, including copyright, in the content of these files and images are owned or controlled for these purposes by Nerdvana Romania.

You may copy, download, store (in any medium), adapt, or modify the content of these Nerdvana Romania resources, provided that you properly attribute the work to Nerdvana Romania.

For any other use of Nerdvana Romania's content, please get in touch with us at office@nerdvana.ro.

© 2024 Nerdvana Romania. All rights reserved.
