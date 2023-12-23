# PID LFR

**Line Follower Robot** or **Line Tracer Robot** or **Robot Tracer** is an Autonomous Robot that follows a visual or magnetic line painted or embedded in the surface of locomotion. Line Follower Robot is a category of a mobile robot that is capable of moving in a surrounding relying on guidance devices that allow them to travel a pre-defined navigation route in relatively controlled space. Some of the Automated Guided Vehicles (AGVs) use magnetic or color tape for the guide path and are usually used in warehouses for autonomously sorting/loading storage racks.

### Contents

- [Acknowledgement](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
- [Images and Videos](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
1. [Basic Theory (PID Algorithm)](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
2. [Hardware Components Required](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
3. [Software Used](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
4. [Hardware Description](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    4.1. [QTR-8RC Line Sensor](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    4.2. [Arduino Nano ATmega328P Board](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    4.3. [TB6612FNG Motor Driver](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    4.4. [N20 Micro Geared Motor](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    4.5. [5V Step-Up Power Module Lithium Battery Charging Protection Board USB (134N3P)](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
5. [Making The Line Follower Bot](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
6. [Circuit Diagram and Code for PID LFR](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    6.1. [Connections](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
    6.2. [Source Code](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)
    
7. [From The Makers](https://www.notion.so/PID-LFR-c41c777236164512b8fc00b2cdd37cc2?pvs=21)

# Acknowledgement

This project would not have been possible without the help and constant guidance of our very supportive seniors, who have been with us throughout the making of this project.

- Respected Mr. Pradum Pal sir (Final [B.Tech](http://B.Tech) Electronics Engineering)
- Respected Mr. Avi Kushwaha sir (Final [B.Tech](http://B.Tech) Mechanical Engineering)
- Respected Mr. Avinash Kumar sir (Second Year M.C.A.)

This is a small note, on behalf of all our team, to thank them for giving their time and efforts for strengthening our skills and enabling us to finally present this project before you all.

We would also like to hand a sincere thanks to the AUTOROB club for giving us this opportunity.

Hope all you guys reap the maximum benefits out of this tutorial!!

# Images And Videos

![NEW1.png](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/NEW1.png)

![NEW2.png](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/NEW2.png)

[LFR_Video.mp4](https://youtu.be/HDnLGCSrQ0g)

In this tutorial we will make a **7cm x 9cm** Line Follower Robot using **Arduino Nano** as the controller, **QTR-8RC** as the sensor, **N20 600RPM 6V DC Gear Motor** as the actuator, **3.7V Li-Ion Battery** with DC-DC boost converter as the power system, and PID based control system as the algorithm to follow the line. The Firmware Code of this project is available for Arduino IDE.

## Basic Theory (PID Algorithm)

A control loop is any system where a feedback mechanism is used to control a certain action.

PID is often used to control these systems. PID is an acronym that stands for Proportional, Integral, and Derivative.

**Proportional**

The first component of the PID algorithm is the simplest to understand and the most crucial to the performance of the controller. The P stands for proportional. It means that the control variable should be adjusted proportionally to the amount of error in the system. A PID algorithm that only utilized a P component could be expressed as:

                                                        *Output=Error*Kp*

                                                                                 where Kp is the proportional coefficient

**Control Variable:** The control variable is the output of the controller that we get to adjust.

**Process Variable:** The process variable is the measured value in the system that you are attempting to control. The process variable is used as feedback to the controller so that it can decide how to adjust the control variable.

**Error:** The error is the difference between the process variable and the set point

The problem with using a proportional-only controller is that it usually results in a steady-state error. As the error gets smaller and smaller, the result of the error multiplied by the proportional coefficient eventually becomes too small to have any affect on the process variable.

**Integral:**

The I stands for integral, which is a mathematical term that means to accumulate something over time*. In the case of PIDs, the I component accumulates the error that occurs over time. That accumulated error is then multiplied by Ki, the integral coefficient, and added to the output. A controller algorithm that only utilized the P and I components could be expressed with the following pseudo-code:

                                            *Accumulation_of_error += error * delta_time*

                                       *Output = (error * Kp) + (accumulation_of_error * Ki)*

The integral component of the control algorithm can remove any steady-state error in the system because it accumulates that error over time and compensates for it, rather than just looking at an instantaneous snapshot of the error at one moment in time.

**Derivative**

The D stands for derivative. A derivative is a mathematical term that means “the slope of a curve.” In this case, the curve is the plot of the error over time. If the error is steady-state, then the result of the D component is zero.  A full PID algorithm could be expressed by the following pseudo-code:

                                           *Accumulation_of_error += error * delta_time*

                                     *Derivative_of_error = (error - last_error) / delta_time*

*Last_error = erroroutput = (error * Kp) + (accumulation_of_error * Ki) + (derivative_of_error * Kd)*

The derivative component is mostly useful when the system is already at or near steady state. At steady state, the P and I components are both very small because the error is very small. If the error suddenly increases it takes a while before the P and I components start kicking in again. However, the D component respond to the sudden change in error and begin compensating immediately. For this reason, it is often said that the D component is responsible for compensating for future error; it sees the error changing and tries to prevent it from changing more.

## Hardware Components Required

- QTR-8RC Line Sensor – 1pc
- Arduino Nano ATmega328P Board – 1pc
- TB6612FNG Motor Driver – 1pc
- 5V Step-Up Power Module Lithium Battery Charging Protection Board USB (134N3P) – 1pc
- Perf Board 7cm x 9cm – 1pc
- BL-5C / BL-4C Li-Ion Battery – 3pcs
- N20 Gear Motor 6V 600rpm – 2pcs
- N20 Motor Wheel 34mm – 2pcs
- N20 Motor Bracket – 2pcs
- Small Caster Wheel – 1pc
- Nut and Bolt – 6pcs
- 0.5mm Wire
- Black Tape
- White Tape
- Tools

## Software Used

- [Arduino IDE](https://www.arduino.cc/en/software)

# Hardware Description

### ****QTR-8RC Line Sensor****

![qtr-rc1.jpg](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/qtr-rc1.jpg)

QTR-8RC Line Sensor

The QTR-8RC is a reflectance sensor array designed primarily to be used as a line sensor for line follower robots. QTR-8RC has eight IR emitter and receiver (phototransistor) pairs evenly spaced at intervals of 0.375″ (9.525 mm). The sensor can be split into two parts if needed. The QTR sensor has eight infrared emitter/receiver that is able to differentiate between a dark surface) with low IR reflectivity) and a light surface (with high IR reflectivity).

What does QTR stand for?

- Q = Charge
- T = Transfer
- R = Resistance

### ****Arduino Nano ATmega328P Board****

![490-4900280_arduino-nano-png.png](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/490-4900280_arduino-nano-png.png)

Arduino Nano

The Arduino Nano is a small, complete, and breadboard-friendly board based on the ATmega328 (Arduino Nano 3.x). It has more or less the same functionality of the Arduino UNO, but in a different package. It lacks only a DC power jack, and works with a Mini-B USB cable instead of a standard one.

**Specification of Arduino Nano**

| Microcontroller | ATmega328P ( AVR Architecture) |
| --- | --- |
| Operating Voltage | 5V |
| Flash Memory | 32KB of which 2KB used by the bootloader |
| SRAM | 2KB |
| Clock Speed | 16MHz |
| Analog IN Pins | 8 |
| EEPROM | 1KB |
| Digital I/O Pins | 22 (6 are PWM) |
| PCB Size | 18mm x 45mm |

### ****TB6612FNG Motor Driver****

![TB6612FNG_motorDriver.jpg](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/TB6612FNG_motorDriver.jpg)

The TB6612FNG Motor Driver can control up to two DC motors at a constant current of 1.2A (3.2A peak). Two input signals (IN1 and IN2) can be used to control the motor in one of four function modes: CW, CCW, short-brake and stop. The two motor outputs (A and B) can be separately controlled, and the speed of each motor is controlled via a PWM input signal with a frequency up to 100kHz. The STBY pin should be pulled high to take the motor out of standby mode.

**Specification of TB6612FNG**

| Supply Voltage VM (Motor) | 4.5 – 13.5V |
| --- | --- |
| Supply Voltage VCC | 2.7 – 5.5V |
| Input Voltage VIN | 3.3V / 5V (IN1, IN2, STBY, PWM) |
| Current Per Channel | 1.2A |
| Current Per Channel Peak Pulse | 3.2A |
| PCB Size | 20mm x 20mm |

**TB6612FNG Pin Operation Logic**

| IN1 | IN2 | PWM | STBY | OUT1 | OUT2 | Mode |
| --- | --- | --- | --- | --- | --- | --- |
| H | H | H/L | H | L | L | Short Brake |
| L | H | H | H | L | H | CCW |
| L | H | L | H | L | L | Short Brake |
| H | L | H | H | H | L | CW |
| H | L | L | H | L | L | Short Brake |
| L | L | H | H | OFF | OFF | Stop |
| H/L | H/L | H/L | L | OFF | OFF | Standby |

### ****N20 Micro Geared Motor****

![N20_gearMotor.webp](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/N20_gearMotor.webp)

N20 6V 600RPM Micro Geared Motor is a tiny motor having a gear ratio of 1:100 is a lightweight high torque motor suitable for a variety of industrial, home appliances, and hobby applications. As compared to other motors of this size either with metal or plastic gears, the N20 metal geared motor has a much higher torque to size ratio.

**N20 6V 600RPM Micro Geared Motor Specification**

| Supply Voltage | 6V |
| --- | --- |
| Gear Ratio | 100:1 |
| No Load Speed @ 6V | 600 RPM |
| No Load Current @ 6V | 0.04A |
| Stall Current @ 6V | 0.67A |
| Stall Torque @ 6V | 0.54 kg-cm |
| Size | 10mm × 12mm × 26 mm |

### ****5V Step-Up Power Module Lithium Battery Charging Protection Board USB (134N3P)****

![5V_stepUpConv-transformed.png](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/5V_stepUpConv-transformed.png)

This 134N3P Charging module has an output of consistent 5V with an input voltage ranging from 3.7V to 5.5V. This module has a built-in MOS for charging and discharging. It has an output current of 1A. The module stops the dispensing power if the input voltage reaches 2.9V. It has a discharge efficiency of 85% with 3.7V input.

It has a maximum of 8µA standby current. It has an over-temperature, overvoltage, overload, and short circuit, as well as overcharging and over-discharging protection. It supports trickle mode, keeping your battery from draining, by charging it equal to its discharge rate. It also supports zero voltage charging.

It could operate in places with temperatures ranging from -30°C to 85°C.

# Making **The Line Follower Bot**

Here are the steps to help you proceed with the making of a PID-LFR.

- First, gather all materials as mentioned in the hardware components section.
- Then on the PCB board start assembling and planning out the placement of components. (If , in any case you prefer buying a zero board that is not the size required, then you will have to cut the right size from it. )
- Now, mount the N20 motors on the chassis.
- Both the motors should be in a straight line and perpendicular to the chassis.
- Cast the castor wheel exactly at the center axis of both the motors .( Make sure that castor wheel is of the exact same height as the wheels so that the bot stays parallel top the ground.)
- The next step is to mount the QTR8 Sensor . The sensor should be placed in such a way that it is not more than 3 mm above the ground surface.
- The sensor should also be parallel to the straight line connecting the motors.
- Now , place the Arduino Nano on the PCB (try to keep the distance between sensor and Arduino as less as possible).
- Now mount all the other electronic components according to your convenience and connect all the components according to the circuit diagram shown (either using soldering techniques or using jumper wires.)

         **** it is advised to connect all the components using soldering to keep the model sturdy.***

- Remember to keep the soldering temperature in the range of 350**°**C to protect the electronic components.

## Circuit Diagram and Code for PID LFR

![circuit diagram.png](PID%20LFR%20c41c777236164512b8fc00b2cdd37cc2/circuit_diagram.png)

### Connections

| QTR-8RC SENSOR | Arduino Nano |
| --- | --- |
| D1 | D9 |
| D2 | D8 |
| D3 | D7 |
| D4 | D6 |
| D5 | D5 |
| D6 | D4 |
| D7 | D3 |
| D8 | D2 |

| TB6612FNG MOTOR DRIVER | Arduino Nano |
| --- | --- |
| PWMA | D10 |
| PWMB | D11 |
| AIN1 | A4 |
| AIN2 | A3 |
| STBY | A2 |
| BIN1 | A1 |
| BIN2 | A0 |

> A1 & A2 → RIGHT MOTOR
B1 &  B2 →LEFT MOTOR
VCC & VM →SHORTED
> 

5V NANO + VCC VM MOTOR DRIVER + VCC QTR8 ARE SHORTED.

ALL THE GROUDS ARE SHORTED.

SWITCH IS CONNECTED WITH THE +VE AFTER THE BOOST CONVERTER.

+VE IS CONNECTED TO THE VCC AND -VE TO THE GROUND.

# From The Makers

We would like to introduce ourselves officially after finishing up this tutorial:

- Aryan Singh (Second [B.Tech](http://B.Tech) Electronics Engineering)
- Shaurya Shukla (Second [B.Tech](http://B.Tech) Civil Engineering)
- Pragya Singh (Second [B.Tech](http://B.Tech) Information Technology)
- Japneet Kaur (Second [B.Tech](http://B.Tech) Computer Science Engineering)

To end up this project , we all as a team, would like to thank all of you for reading this tutorial. We  hope that you grasp some solid concepts from this tutorial and it will help you in more ways than one.

Thank you all for bearing along!!

All the best for your future!!

              ****
