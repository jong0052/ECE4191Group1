## ECE4191 Integrated Design Group 1:
### Introduction
This is the working dirctory of 2024 Monash ECE4191 Integrated Design - Group 1. The goal of the unit is to design a package delivery robot that is capable of coorporating with robot from another group to achieve the task of delivering cubic packages of various size into three arbitary locations. 

![Final Assembly](https://github.com/jong0052/ECE4191Group1/blob/main/readmeimages/0_full_assembly.PNG )

![Electrical System](https://github.com/jong0052/ECE4191Group1/blob/main/readmeimages/1_electrical_system.jpg)

**For the full Comprehensive Project Report, please view: [Full Report](https://github.com/jong0052/ECE4191Group1/blob/main/ECE4191_Team_1_Final_Report.pdf)**

### File System Description

1. **Folder - arduino:**  Folder contains Arduino files which will be pushed into RPi-pico and Arduino Nano.
    - **Folder - Pico-10DOF-IMU_Rev2_1:** Folder containing code for IMU, ToF, need to be pushed into RPI-pico.
    - **Folder - PID_speed_control:** Folder containing code for PID Motor controller, need to be pushed into Arduino Nano.
    - **Folder - serial_test_arduino:** Folder containing code for testing serial communication between RPI and Arduino Nano.

2. **Folder - Communication:** Folder contains necessary communication file which are needed for managing bluetooth communication between two robots. 

3. **Folder - homography_rooftop_nav:** Folder containing test as well as deployable code which are needed for the roof-top honography navigation, unfortunately due to the roof-top at the competition environment does not contain sufficient distinguishable features, the code is included in our final design. 

4. **Folder - legacy:** Folder contains legacy code that are not used in the final design.
    - **File - navigation_loop_rrtc.py:** Legacy code for the rrt planner which due to its unstability is abandoned for the final design. 
    - **File - Servo.py** Legacy code for servo that are not functioning as expected. 
    - **File - ultrasonic_loop.py:** Legacy code for ultrasonic sensors which will no longer be used due to better sensors (ToF) are implemented. 

5. **Folder - test_code:** Folder containing all the test code for each subsystems. 
    - **Folder -Testing Code Sensor:** Folder containing the testing code for I2C communication between sensors (IMU, ToF) and the microcontroller (RPi-pico).
    - **File - multiprocess_class_test.py, multiprocessing_test.py:** Files for testing the multiprocessing capability of RPi. 
    - **File - serial_test.py:** File for testing the Serial communication (USB) between Arduino Nano and RPi.
    - **File - ultrasonic_cal.py:** File for calibrating the Ultrasonic Sensors (which are abandoned for the final design).
    - **File - K.py** File for kalman filter abandoned due to being unnecessary.

6. **Folder - utils:** Folder containing the code for implementing different algorithms (this include Astar, MPC, Obstacle etc.).

7. **File - bluetooth_test_client.py, bluetooth_test_host.py:** the test client and host code which will be provided to the two parties in the arena to establish communication between two devices. 

8. **File - main.py:** the main environment for operating, controlling and observing the robot.

9. **File - navigation_loop.py:** File containing the navigation module of the robot.

10. **File - plotting_loop.py:** File containing the system-user interface of the robot, will plot out all where the system thinks the robot is. 

11. **File - RFID.py:** File containing the hardware interface for detecting RFID tags, which will identify the location where the package need to be delivered to.

12. **File - serial_loop_gyro_tof.py:** File containing the module which allows for serial communication between RPi and RPi-Pico.

13. **File - serial_loop.py:** File containing the module which allows for serial communication between RPi and Arduino Nano.

14. **File - ServoNew.py:"** File containing the Servo module which will allow for the closing and opening of the package delivery system.

### Wiring Diagrams
![Wiring Diagram 1](https://github.com/jong0052/ECE4191Group1/blob/main/readmeimages/2_elec_diag_1.PNG )

![Wiring Diagram 2](https://github.com/jong0052/ECE4191Group1/blob/main/readmeimages/3_elec_diag_2.PNG)

![Wiring Diagram 3](https://github.com/jong0052/ECE4191Group1/blob/main/readmeimages/4_elec_diag_3.PNG )

![Wiring Diagram 4](https://github.com/jong0052/ECE4191Group1/blob/main/readmeimages/5_elec_diag_4.PNG )


