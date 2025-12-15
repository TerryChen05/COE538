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
<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/5b5f60fa-b391-4d97-93c6-65cf65009986" />


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

## Final Project: Robot Guidance Challenge

**Objective:** Program the eebot to use Cadmium Sulphide photoresistors to autonomously navigate a maze through line tracking and state machine logic.

### Project Requirements
- Implement a working line tracker guiding algorithm
- Detect dead ends/barriers and make branching decisions at junctions
- Navigate a maze with S turns and L/T junctions
  - S turns: curved track
  - L/T junctions: 2-3 path option intersections 

### Implementation Details
1. **5-Sensor Array Configuration**:
   - Sensor A (BOW): Front center sensor for forward alignment
   - Sensor B (PORT): Left sensor for junction detection
   - Sensor C (MID): Center tracking sensor for general alignment
   - Sensor D (STBD): Right sensor for junction detection
   - Sensor E/F (LINE): Alignment sensor for left/right drift detection
   
   <img width="400" height="310" alt="Sensor Array Layout" src="https://github.com/user-attachments/assets/ced37e9a-d9f7-44db-b94a-9069c29274f5" />
   
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Fig. 1: Bird’s Eye View of the Sensor Configuration
    
2. **Sensor Calibration**:
   - CdS sensors have high resistance in darkness, and low resistance when illuminated
   - Base dark values (sensor over black track line) and corresponding tolerance thresholds for each sensor
     <img width="440" height="180" alt="image" src="https://github.com/user-attachments/assets/4ecd2b73-29fd-437c-98f4-b0273425425d" />

3. **State Machine Architecture**:
   
    | State       | State # | Description                                     |
    |-------------|------|-------------------------------------------------|
    | START       | 0    | Initial state; waits for front bumper trigger to begin |
    | FWD         | 1    | Forward motion with active line tracking and junction detection |
    | ALL_STOP    | 2    | Emergency stop triggered by rear bumper        |
    | LEFT_TRN    | 3    | Executing left turn at junction                |
    | RIGHT_TRN   | 4    | Executing right turn at junction               |
    | REV_TRN     | 5    | 180° reversal after hitting dead end           |
    | L_ALIGN     | 6    | Post-left-turn alignment to center on line     |
    | R_ALIGN     | 7    | Post-right-turn alignment to center on line    |
4. **Navigation Logic**:
    1. **Strategy**:
       - The robot follows a **left-priority** exploration strategy (always attempts left turn first)
       - At right-facing L junctions: Continues forward if no left option exists
       - At dead ends, execute a 180° turn and retrace the path
    2. **Line Following**:
       - Continuous sensor polling (multiple times per second)
       - Proportional steering corrections based on sensor readings
       - Detects when sensors A and C are aligned (within tolerance) with the track
       - Corrects for left/right drift using sensor E/F readings 
    3. **Junction Detection**:
       - Detects L and T junctions when sensors B or D detect line presence
       - Executes a series of partial turns to navigate branches (left -> fwd -> right branch priority)
       - Finishes turns with alignment checks
    4. **Dead End Recovery**:
       - Front bumper collision triggers REV_TRN state
       - Executes a 180° turn sequence by briefly reversing before performing a right turn, ending when the BOW is aligned with the track 





**File:** `COE538_FinalProject/maze_solving_robot/Sources/main.asm`

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
