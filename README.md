# Ball and Beam Fuzzy PID Control

## Project Overview

This project implements a Ball and Beam control system using a hybrid Fuzzy Logic and PID control architecture.

The objective is to maintain a ball at a desired position on a beam by continuously adjusting the beam angle through a DC motor.

The system combines:

- Fuzzy Logic Controller (Outer Loop)
- PID Controller (Inner Loop)
- HC-SR04 Ultrasonic Sensor
- Potentiometer Feedback
- DC Motor Actuation
- Arduino Uno

---

## Control Strategy

### Outer Loop – Fuzzy Logic

The fuzzy controller determines the desired beam angle based on:

- Position Error
- Error Velocity

The controller uses membership functions and fuzzy inference rules to generate a target beam angle.

### Inner Loop – PID Controller

The PID controller tracks the desired beam angle.

Inputs:

- Desired Angle
- Actual Angle

Output:

- Motor Command

PID gains:

- Kp = 0.35
- Ki = 0.03
- Kd = 0.05

---

## Hardware Components

- Arduino Uno
- HC-SR04 Ultrasonic Sensor
- Linear Potentiometer
- DC Motor
- Motor Driver
- Power Supply

---

## Pin Configuration

### Ultrasonic Sensor

| Signal | Pin |
|----------|----------|
| Trigger | 5 |
| Echo | 6 |

### Potentiometer

| Signal | Pin |
|----------|----------|
| Output | A5 |

### Motor Driver

| Signal | Pin |
|----------|----------|
| PWM | 9 |
| IN1 | 10 |
| IN2 | 11 |

---

## Features

- Real-time distance measurement
- Sensor filtering
- Fuzzy inference system
- PID stabilization
- PWM motor control
- Deadband compensation
- Position regulation

---

## Future Improvements

- Adaptive PID tuning
- Kalman filtering
- Encoder feedback
- Machine Learning optimization
- Remote monitoring

---

## Author

Lea Mehmetaj

Faculty of Mechatronics Engineering

UBT College

2026
