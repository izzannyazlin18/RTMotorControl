# RTMotorControl

A robust, real-time motor control application developed to handle precise motor actuation, speed regulation, and hardware interface management.

## 🚀 Features
* **Real-Time Control:** Low-latency execution loop for stable motor responsiveness.
* **Modular Codebase:** Clean separation of hardware interfacing and control logic.
* **Closed-Loop Regulation:** Implements a deterministic PI control loop with encoder feedback for precise speed correction.

## 🛠️ Hardware & Tools
* **Embedded Hardware:** NI myRIO-1900 Embedded Evaluation Device
* **Actuation System:** DC Motor with an attached Load Flywheel/Wheel
* **Power Electronics:** Dedicated Motor Driver Interface
* **Development Environment:** LabVIEW Real-Time

## 📂 Repository Structure
* `main.*` - The primary execution script containing the core control loop and hardware configurations.
* `README.md` - Documentation and project overview.

## ⚡ Quick Start
1. Connect the hardware components according to your circuit diagram.
2. Open the `main.*` file in your preferred development environment.
3. Flash/compile the code onto your hardware platform.
