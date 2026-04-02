# GroupA-TeamA3
Our subgroup

# Project: BattleBot Line Maze

---

## Project Overview
The **GripperBot-PD** is designed to complete a "Race Day" course that involves object detection, mechanical retrieval, and precision navigation. The robot remains stationary until triggered, then autonomously grabs a target and follows a complex track consisting of straightaways, 90-degree junctions, and a designated finish zone.

### Core Capabilities:
* **Detection:** HC-SR04 Ultrasonic sensor for hands-free start triggering.
* **Manipulation:** Servo-driven gripper for securing and carrying payloads.
* **Navigation:** 8-sensor IR array managed by a **Proportional-Derivative (PD)** controller.
* **Feedback:** 4-pixel NeoPixel telemetry for real-time state visualization.

---

## Hardware Pin Mapping
To ensure the code functions correctly, the hardware must be wired according to the following configuration:

### Control & Actuators
| Component | Arduino Pin | Role |
| :--- | :--- | :--- |
| **Left Motor (A1)** | 11 | PWM / Backward |
| **Left Motor (A2)** | 10 | PWM / Forward |
| **Right Motor (B1)** | 9 | PWM / Forward |
| **Right Motor (B2)** | 6 | PWM / Backward |
| **Gripper Servo** | 12 | PWM Signal |

### Sensors & Telemetry
| Component | Arduino Pin | Role |
| :--- | :--- | :--- |
| **NeoPixel DIN** | 7 | Data Signal (4 Pixels) |
| **Ultrasonic TRIG** | 4 | Pulse Trigger |
| **Ultrasonic ECHO** | 13 | Pulse Return |
| **IR Sensor 0-7** | A0 - A7 | Line Position Input |

---

## Logic Structure (The State Machine)
The robot's behavior is divided into distinct "States" to ensure reliable performance:

1.  **WAIT_FOR_START:** Scans for obstacles < 20cm via Ultrasonic.
2.  **CONE_STATE/GRAB:** Timed forward drive and gripper closure.
3.  **FOLLOWING:** Active PD line tracking (Smooth steering).
4.  **CROSSING/TURNING:** Navigation through 90° intersections.
5.  **RECOVER:** "Search" mode if the robot loses the line.
6.  **FINISH/BACKUP:** Detects all-black line, parks, and shuts down.

---

## Performance Tuning
These constants are the "knobs" used to adjust the robot's behavior during the Race Day module:

* `KP_GAIN`: Controls how aggressively the robot reacts to the line.
* `KD_GAIN`: Smoothes out the turns to prevent shaking/oscillation.
* `SPEED_BASE`: The standard cruising speed for the motors.
* `LINE_THRESHOLD`: The sensitivity for distinguishing the line from the floor.

---

## Weekly Progress & Portfolio
Detailed documentation, logs, and code iterations for each week of the module:

* **[Week 1: Initial Setup & Hardware](https://github.com/CalebGaitou/GroupA-TeamA3/tree/main/Week%201)**
* **[Week 2: Sensor Integration](https://github.com/CalebGaitou/GroupA-TeamA3/tree/main/Week%202)**
* **[Week 3: Logic & State Machine](https://github.com/CalebGaitou/GroupA-TeamA3/tree/main/Week%203)**
* **[Week 4: Gripper & Actuators](https://github.com/CalebGaitou/GroupA-TeamA3/tree/main/Week%204)**
* **[Week 5: PID Tuning & Optimization](https://github.com/CalebGaitou/GroupA-TeamA3/tree/main/Week%205)**
* **[Week 7: Final Testing & Refinement](https://github.com/CalebGaitou/GroupA-TeamA3/tree/main/Week%207)** 

---
*Created by Group A - Team A3 Caleb Gaïtou & Ngangfor*