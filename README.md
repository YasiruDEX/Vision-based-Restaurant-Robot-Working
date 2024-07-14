
**Robot LUNA** is a state-of-the-art restaurant waiter robot, designed with a focus on stability and precision. It is equipped with a dual camera setup, including an HD wide-angle camera and a Kinect-2 depth camera, enabling vision-based navigation in a 3D restaurant environment grid. The robot's enhanced stability ensures the safe delivery of food and drinks without spillage, even during braking, thanks to a dedicated stabilization tray circuit LUNA will ensure safety. Internally, LUNA has three Raspberry Pis for separate parallel processing tasks and is connected to custom PCB based on Atmega2560 to get stable and accurate sensor and encoder readings, and to control motor drivers accurately. Externally it communicates with a Restaurant's local server computer with ROS 1 Noetic through the local wifi network ensuring precise navigation path and food order locations based on the restaurant tables. This combination of advanced technology and innovative design makes LUNA a reliable and efficient addition to any restaurant staff.

> Tools & technologies used: 
- ROS 1 Noetic Ninjemys, 
- TensorFlow, 
- OpenCV, 
- Open3D,  
- C++ with Atmega2560 custom PCB for Motor and Stability Control, 
- Python with Raspberry Pi 4b boards; (3 SBCs),
- Kinect v2 depth camera and a 150 Degree Wide Angle HD camera.

> Team Members:
- [Hasitha Gallella](https://github.com/HasithaGallella)
- [Sandun Herath](https://github.com/sandun21)
- [Yasiru Basnayaka](https://github.com/YasiruDEX)
- [Lasith Haputhanthri](https://github.com/lasithhaputhanthri)
- [S Thamirawaran](https://github.com/Thamirawaran)

Demonstration VideoLink: https://

- With Robot LUNA at Sri Lanka Robotics Challenge 2024 (SLRC 24)

![LUNA_atSLRC](https://github.com/LUNA-Vision-based-Restaurant-Robot/.github/assets/111054736/354b1bcc-fff9-4360-9827-44e59350e603)



## Robot LUNA System Overview

The main components of the LUNA Restaurant Robot are illustrated in the block diagram below. This includes the central restaurant computer, local WiFi network, robot's computer unit, motor controller unit, stabilization tray controller unit, cameras, and power unit.

![Main Block Diagram of LUNA Restaurant Robot](https://github.com/LUNA-Vision-based-Restaurant-Robot/.github/blob/main/images/SubUnits_Diagram.jpg?raw=true)


## LUNA Sub System Explanation

1. **Central Restaurant Computer**: Manages overall operations and communicates with the robot via the local WiFi network.
2. **Wide Angle Camera**: Provides a broad view of the robot's surroundings for navigation and obstacle avoidance.
3. **Kinect v2 Depth Camera**: Captures depth information to help the robot understand its environment in three dimensions.
4. **Motor Controller Unit**: Controls the movement of the robot's wheels and ensures precise navigation.
5. **Stabilization Tray Controller Unit**: Maintains the stability of the tray to prevent spillage while the robot is moving.
6. **Power Unit**: Supplies power to all components of the robot, ensuring uninterrupted operation.

## PCB Details

The LUNA robot's functionalities are controlled by a custom-designed PCB which includes two main units: the Motor Controller Unit and the Stabilization Tray Controller Unit. Below are the front and back views of the PCB:

<table>
  <tr>
    <td><img src="https://github.com/LUNA-Vision-based-Restaurant-Robot/.github/blob/main/images/PCB_Front.jpg?raw=true" alt="PCB Front View" width="400"/></td>
    <td><img src="https://github.com/LUNA-Vision-based-Restaurant-Robot/.github/blob/main/images/PCB_Back.jpg?raw=true" alt="PCB Back View" width="400"/></td>
  </tr>
  <tr>
    <td align="center">PCB Front View</td>
    <td align="center">PCB Back View</td>
  </tr>
</table>

Special thanks to Dr. Ranga Rodrigo, Prof. Rohan Munasinghe, and Prof. J.A.K.S. Jayasinghe for their guidance and support throughout our journey.
Also we would like to extend our special gratitude to [Department of Electronic and Telecommunication Engineering, University of Moratuwa](https://ent.uom.lk/) for the invaluable support to the project. 
