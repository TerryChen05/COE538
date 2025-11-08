# COE538 - Microprocessor Systems

This repository contains lab assignments and the final project(WIP) for COE538 completed in Fall 2025. The course focuses on programming the HCS12 microcontroller (9S12C32) for the eebot (a mobile robot) using assembly language.

> **Note:** Labs 1 and 2 focused on fundamental assembly programming concepts, serving primarily as introductory exercises, with the later labs directly building on the introduced topics, so I haven't included them in this repository.

---

## Lab 3: Battery and Bumper Displays

**Objective:** Develop software to read analog inputs and display formatted data on the eebot LCD.

### Features Implemented/Key Topics
- A/D converter initialization and reading from multiple channels
- Fixed-point arithmetic for battery voltage calculation
- Binary-to-BCD conversion for voltage display
- BCD-to-ASCII conversion with leading zero suppression
- Bumper switch state detection via digital inputs (AN02, AN03)
- Real-time display of battery voltage (formatted as "x.x [V]") and bumper status

**File:** `COE538_lab3/lab3_eebot/Sources/main.asm`  

---

## Lab 4: Motor Control and Hardware Timer

**Objective:** Create motor control routines, and implement an interrupt-driven timing system driven by the system clock.

### Features Implemented/Key Topics
- Motor control subroutines:
  - Speed control (ON/OFF) for port and starboard motors
  - Direction control (FWD/REV) for both motors
- Timer overflow interrupt service routine
- 8-bit software clock running at 23 Hz (43.7 ms resolution)
- Timed alarm system using Timer Overflow counter
- LCD output based on Timer alarms

**Files:**  
    `COE538_lab4/MotorTest/Sources/main.asm`  
    `COE538_lab4/TimerOverflow/Sources/main.asm`  
    `COE538_lab4/LCDtimer/Sources/main.asm`

---

## Lab 5: Robot Roaming Program

**Objective:** Design and implement a state machine-based robot navigation system.

### Features Implemented/Key Topics
- State machine design with 6 states:
  - START: Initial state, waits for front bumper trigger
  - FORWARD: The eebot drives forward for a predesignated amount of time
  - REVERSE: Backs up after obstacle collision (forward bumper triggered)
  - FORWARD_TURN: Executes roaming turn
  - REVERSE_TURN: Turns after backing up
  - ALL_STOP: Stops all motors (rear bumper trigger)
- State dispatcher for centralized state management
- Obstacle avoidance using front bumper detection
- Timed maneuvers for turns and reversals
- Real-time state display on LCD (battery voltage and current state)
<img width="410" height="505" alt="image" src="https://github.com/user-attachments/assets/334ff143-cb4f-45f3-81fc-5261d844f9b1" />

**File:** `COE538_lab5/state_machine/Sources/main.asm`  

---

## Final Project: Robot Guidance Challenge (WIP)

**Objective:** Program the eebot to autonomously navigate a maze, learn the correct path, and retrace it.

### Project Requirements
- Navigate a maze with S turns and L/T junctions
- Detect dead ends/barriers and make branching decisions at junctions
- Store maze solution in memory
- Retrace correct path on return journey

### Implementation Details
- WIP

**File:** `N/A`

---

## Technical Specifications

**Development Environment/Hardware:**
- Microcontroller: MC9S12C32 (HCS12 family)
- CodeWarrior IDE
- Assembly Language (HC12)
- Serial Monitor for debugging

---

## Course Information

**Institution:** Toronto Metropolitan University (formerly Ryerson University)  
**Course:** COE538 - Microprocessor Systems  
**Term:** Fall 2025  
