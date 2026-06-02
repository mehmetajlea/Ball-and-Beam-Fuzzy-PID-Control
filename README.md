# Ball and Beam Control System Using Fuzzy Logic and PID Control

## Course
Fuzzy Logic and Control

---

## Project Description

This project presents the design and implementation of a Ball and Beam control system using a hybrid Fuzzy Logic and PID control architecture on an Arduino Uno platform.

The objective is to maintain a ball at a desired position on a beam by continuously adjusting the beam angle through a DC motor. The system combines intelligent decision-making using fuzzy logic with precise actuator control through a PID controller.

The project demonstrates the practical application of fuzzy logic, classical control theory, sensor fusion, and embedded systems programming in a real-time mechatronic system.

---

## System Overview

The control architecture consists of two cascaded control loops:

### Outer Control Loop – Fuzzy Logic Controller

The fuzzy controller determines the desired beam angle based on:

- Ball position error  
- Ball velocity (error rate)

Inputs:

\[
e = x_{target} - x_{actual}
\]

\[
\dot{e} = \frac{de}{dt}
\]

Output:

\[
\theta_{target}
\]

The fuzzy inference system uses membership functions and rule-based reasoning to generate the optimal beam angle for stabilizing the ball.

---

### Inner Control Loop – PID Controller

The PID controller regulates the beam angle by minimizing the error between desired and actual angle.

Error:

\[
e_{\theta} = \theta_{target} - \theta_{actual}
\]

Control Law:

\[
u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}
\]

Controller Gains:

| Parameter | Value |
|------------|--------|
| Kp | 0.35 |
| Ki | 0.03 |
| Kd | 0.05 |

---

## Hardware Components

### Main Controller
- Arduino Uno

### Sensors
- HC-SR04 Ultrasonic Sensor
- Potentiometer (Rev-type feedback)

### Actuation
- DC Motor
- L298N Motor Driver

### Power System
- External DC Power Supply

---

## Pin Configuration

### Ultrasonic Sensor
| Signal | Arduino Pin |
|--------|-------------|
| Trigger | D5 |
| Echo | D6 |

### Potentiometer
| Signal | Arduino Pin |
|--------|-------------|
| Analog Output | A5 |

### Motor Driver
| Signal | Arduino Pin |
|--------|-------------|
| PWM | D9 |
| IN1 | D10 |
| IN2 | D11 |

---

## Software Features

- Real-time distance measurement
- Low-pass sensor filtering
- Fuzzy inference system
- PID control loop
- PWM motor control
- Deadband compensation
- Noise reduction filtering
- Smooth angle tracking
- Error rate estimation

---

## Fuzzy Logic Design

### Input Membership Functions
- Negative
- Zero
- Positive

### Output Membership Functions
- Low Angle
- Flat Angle
- High Angle

The controller is based on a rule set of 9 fuzzy rules for decision-making.

---

## Experimental Objective

- Target Ball Position: 20 cm  
- Beam Operating Range: 132° – 158°  
- Neutral Angle: 145°  

---

## Future Improvements

- Adaptive PID tuning  
- Kalman filtering for noise reduction  
- Encoder-based feedback system  
- Machine learning-based optimization  
- Wireless monitoring and telemetry  
- Data logging and system identification  
- Model Predictive Control (MPC)  

---

## Applications

This system is applicable in:

- Control Systems Engineering  
- Mechatronics Engineering  
- Robotics  
- Embedded Systems  
- Intelligent Control Systems  
- Industrial Automation  
- Fuzzy Logic and Control Systems  

---

## Author

Lea Mehmetaj  
Bachelor of Mechatronics Engineering  
UBT College, Prishtina  
2026
