# iai_pcon_driver  
A ROS2 node to operate IAI Robocylinder using the PCON controller series.  

## Tested Environment  
Cable  : Dtech RS232C to USB converter cable(PL2303)  or IAI official RS232C to USB converter cable included in IA-OS-C  
RS232 Interface : SIO converter for PCON-CB/CFB  
Controller : PCON-CFB  
Robocylinder : RCP6-SA8C  

## Setup  
It could support multiple baudrate, but I highly recommend to setup the baudrate to 230400 baud.  
You need IA-OS-C software in windows to setup for that.  

## How to run  
```bash
ros2 run iai_pcon_driver pcon_ros_node  
```

## Topics
### Publisher  
/joint_states (sensor_msgs/JointState)  
currently, it only supports position output. TODO : velocity  
unit is based on SI and ROS Standards [m, m/s]  
 
### Subscriber 
/target_pose  (std_msgs/Float64)  
unit : mm, able to support 0.1mm resolution   

### Parameters  
baudrate (float), default 230400  
speed_mms (float), default 50.0mm/s  : speed  
acceleration_g(float), default 0.3G  : acceleration 
publish_rate_hz(float), default 50Hz  

### Publish
```bash
# Move to 500mm
ros2 topic pub --once /target_pose std_msgs/msg/Float64 "{data: 500.0}"
```