# **FOC Controller for BLDC Motors**  
A portable, self-contained closed-loop Field Oriented Controller (FOC) for Brushless DC (BLDC) motors.  

This project focuses on the development of a robust FOC controller for BLDC motors to be integrated into future robotic systems, such as robotic arms, legs, and biped robots. The controller combines the advantages of different motor technologies, offering:  
- The **speed** of a BLDC motor,  
- The **precision** of a stepper motor,  
- The **torque** provided by planetary gear reductions.

---

## **Features and Goals**  
- **High Performance**: Closed-loop field-oriented control for precise speed, position, and torque management.  
- **Integrated Protection**: Includes reverse voltage, overcurrent, undervoltage, and current spike protections for reliability.  
- **Modularity**: Designed for portability and adaptability across multiple robotic platforms.  
- **Advanced Sensors**: Leverages IMU and magnetic sensors for precise motor orientation and axis positioning.

---

## **Hardware**  

### **Core Components**  
- **Microcontroller**:  
  - STM32 or ESP32, serving as the main processing unit.  
- **Sensors**:  
  - **IMU (MPU9020)**: For 3D acceleration and gyroscope tracking.  
  - **Magnetometer**: For accurate spatial awareness and heading.  
  - **Magnetic Sensor**: Paired with a diametrically magnetized rotor to measure the motor shaft angle precisely.  

### **Protection Features**  
- Reverse voltage protection.  
- Overcurrent and undercurrent detection.  
- Current spike suppression for stability.  

---

## **Firmware**  
The firmware leverages the **SimpleFOC** library for Arduino, a widely adopted solution for BLDC motor control.
Planned enhancements include advanced tuning parameters and optimizations for specific robotic applications.  

---

## **Software**  
The software ecosystem includes:  
1. **Motor Calibration Tools**: For initial setup and alignment.  
2. **Real-Time Monitoring**: Visualization of motor parameters (speed, torque, and position).  
3. **Control Interface**: A graphical interface for setting PID gains, operating modes, and safety limits.  

Integration with robotics frameworks (e.g., ROS2) is under consideration to enable easy adoption in robotic projects.  

---

## **Mechanical Design**  
The mechanical system will include:  
- **Planetary Gearbox**: For torque amplification and precission gain.  
- **Custom PCB**: Designed to ensure compact integration of the motor, sensors, and controller.  
- **Reusability**: The controller is designed to fit in not only BLDC motors but also stepper ones.  

---

## **Applications**  
- **Robotic Arms and Hands**: Precise positioning for manipulation tasks.  
- **Bipedal Robots**: Dynamic motion control for walking and balancing.  
- **Legged Robots**: Torque-rich actuators for smooth and powerful movement.  

---

## **References and Inspiration**  
- [FOCControl Library for Arduino](https://github.com/simplefoc/simplefoc)  
- https://www.youtube.com/@QihaoGuo
- https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html
- Research papers on advanced FOC methodologies and BLDC motor optimizations.
