# RTMotorControl
## DC Motor Velocity Control via myRIO Linux RT

A real-time embedded C application targeting the NI myRIO hardware platform to implement closed-loop velocity control on a DC motor. This project utilizes hardware timer interrupts (IRQ) for precision tracking and a custom cascading biquad filter implementation for control loop processing.

## 📂 Repository Contents

* `RTMotorControl.c` - Core implementation containing the main loop, real-time Timer ISR (Interrupt Service Routine), encoder evaluation (`vel`), and the cascading biquad filter logic.
* `README.md` - Documentation and project architecture overview.

## 🛠️ System Architecture & Dependencies

This codebase is designed to compile inside an NI cross-compilation toolchain environment (such as Eclipse for NI myRIO). 

To successfully compile this project, the environment requires access to the following dependencies, which are omitted from this repository due to licensing and environment constraints:

### 1. National Instruments Hardware Abstraction Layers
* `MyRio.h` & `NiFpga.h` — NI core libraries handling FPGA session management, memory-mapped registers, and low-level I/O.
* `AIO.h` — Analog Input/Output interface utilities.
* `TimerIRQ.h` & `IRQConfigure.h` — Real-time hardware interrupt register configurations.

### 2. Embedded Utilities & Course Libraries
* `T1.h` — Hardware specific type definitions.
* `ctable2.h` — Interactive command-line parameter tuning user interface (`ctable2`).
* `matlabfiles.h` — High-level stream handlers used to parse and save runtime buffers directly into native MATLAB `.mat` workspace files.

### 3. Standard Linux Libraries
* `pthread.h` — Standard POSIX threads library for handling asynchronous IRQ thread scheduling.
* `math.h` — Core mathematics library utilized for floating-point calculations (`M_PI`, etc.).

## 🚀 Key Implementation Details

* **Asynchronous ISR Execution:** The application registers a dedicated hardware timer loop, spawning a low-latency thread via `pthread_create` to prevent blocking the terminal interface menu loop.
* **Biquad Cascade Filtering:** Contains a hand-rolled difference equation solver (`cascade`) with output saturation logic (`SATURATE`) running a multi-segment biquad framework for real-time error corrections.
* **Data Logging:** Streams system parameters (`Omega_J`, `VDAout`) straight to double-precision static ring buffers, exporting them into a target `.mat` binary file upon proper thread termination (`pthread_join`).
