# Portrait-Drawing Robotic Arm

A 3-DOF robotic arm that uses inverse kinematics to draw sketches on paper. Built during BlueStamp Engineering, Summer 2026.

## Overview

This project combines mechanical design, embedded control, and computer vision into a robotic arm that can take an image and reproduce it as a pen drawing. The arm uses inverse kinematics to translate pixel coordinates into servo angles, producing smooth drawing motions on paper.

## How It Works

1. An image is processed and converted into line-drawing paths
2. The inverse kinematics solver converts each (x, y) point into shoulder and elbow servo angles
3. Servo commands are sent over serial to an Arduino controlling three servos
4. A joystick provides manual control for calibration and testing

## Tech Stack

- **Python** - inverse kinematics solver, drawing path generation
- **Arduino/C++** - servo control, joystick input handling
- **Hardware** - 3 servo motors, Arduino, joystick module, pen holder

## Built at BlueStamp Engineering

This project was developed as part of the BlueStamp Engineering summer program. See the full code and technical details at [github.com/samjm2/portrait-arm](https://github.com/samjm2/portrait-arm).
