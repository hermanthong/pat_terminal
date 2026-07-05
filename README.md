# PAT Terminal Design Document

This repo models the on-board software for a laser communications Pointing, Acquisition and Tracking (PAT)
terminal. The terminal has to acquire a counterpart terminal, lock onto it, and hold a tight pointing lock while both platforms move and vibrate.

### Deployment
This repo ships a Dockerfile. Uses ROS2 Humble.

### Glossary
- **Spot** is the beacon's image on the detector.
- **Position error** is the spot's offset from the detector center.
- **Handoff** is the transfer of pointing authority from the coarse loop to the fine loop.
#### Sensors and Actuators
- ***Camera** sees the spot and gives a position error.
- **IMU** is the Inertial Moment Unit. It provides angular rate and attitude data.
- **Gimbal** is the hardware in control of the coarse loop.
- **FSM** is the Fast Steering Mirror. It is the hardware in control of the fine loop. Not to be confused with "finite state machines".

## Part A: System Architecture
### A.1: Assumed Hardware Numbers
| Device | Assumed characteristics |
|---|---|
| Camera / detector | 60 fps, ~30 ms total latency |
| IMU | 1 kHz angular rate, < 1 ms latency |
| Gimbal | max rate ~20°/s, closed-loop bandwidth ~5 Hz |
| FSM | ±1 mrad optical range, ~1 kHz-class bandwidth, commanded at 1 kHz |

### A.2 Node Architecture


### A.3 Control Hierarchy

### A.4 Timing and latency budget

### A.5 ROS2 middleware choices

### A.6 Failure Handling

## Part B: The hardest part

## Part D: Design Considerations
