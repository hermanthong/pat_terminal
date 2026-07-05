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

### A.2 Node architecture

```mermaid
flowchart LR
    subgraph Sensors [Sensors]
        CAM[camera_node<br/>-position error, 60 Hz]
        IMU[imu_node<br/>-angular rate, 1 kHz]
    end

    FINE[fine_controller<br/>-estimator<br/>-FSM control law, 1 kHz]

    COARSE[coarse_controller<br/>slew / offload, 100 Hz]
    MODE[mode_manager<br/>state machine, 60 Hz]
    TEL[telemetry_node<br/>health + telemetry aggregation, 10 Hz]
    HOST[host_link<br/>mothership bridge]

    CAM -- "/position_error" --> FINE
    IMU -- "/imu_data" --> FINE
    FINE -- "/fsm_cmd" --> ACT1[(FSM)]
    FINE -- "/fsm_state" --> COARSE
    COARSE -- "/gimbal_cmd" --> ACT2[(Gimbal)]
    CAM -- "/position_error" --> MODE
    FINE -- "/fsm_state" --> MODE
    MODE -- "/mode" --> FINE
    MODE -- "/mode" --> COARSE
    HOST -- "set_mode (service)" --> MODE
    MODE -- "/mode" --> TEL
    TEL -- "/telemetry" --> HOST
```

#### camera_node
- Frame capture
- Computation of spot centroid
- Handles a validity check (no spot / saturated / on edge).
- Publishes position error in angular units, stamped with exposure time
- **Consideration**: exposure time should be used over publish time for accuracy's sake

#### imu_node
- IMU driver
- Handles calibration transformations
- Publishes calibrated angular rates and attitude at 1 kHz
- **Consideration**: There should not be any filters here. It is the estimator's job to calculate the beam's pointing error at any given time stamp, which is fused from the IMU + camera data.

#### estimator
- Answers one question: where is the beam pointing right now?
- Complementary filter: propagates the error estimate from IMU rates every 1 ms, then
  nudges it toward each camera measurement with a low-gain correction as frames arrive
- The two sensors fail in opposite frequency bands (IMU drifts at DC, camera cannot see
  vibration); this is the minimal fusion that exploits that
- A missing or invalid camera frame means no correction that cycle — the estimate
  coasts on IMU propagation, which is the same mechanism COAST relies on
- O(1) fixed-cost arithmetic per cycle, no allocations — fit for the 1 kHz path
- **Consideration**: this is a component inside fine_controller, not a node of its
  own — the estimate feeds the control law at 1 kHz and must not cross a process
  boundary (see A.5)
  
#### fine_controller
- Owns the FSM
- Owns the pointing-error estimate
- Publishes its own deflection state for the offload loop and for mode supervision

#### coarse_controller
- Owns the gimbal
- Slews to the commanded bearing
- Offload
  
#### mode_manager
  - Owns the state machine and is the single writer of `mode`
  - Exposes `set_mode` for the host to command
  - **Consideration**: `set_mode` is a service because the host must know if the command is accepted or rejected.
  - **Consideration** `mode` is distributed as a topic, not many per-node services. It is state that many nodes need, including ones that weren't alive when it changed (reliable + transient-local delivers the current mode to late joiners). Instead, mode_manager supervises behavior (`fsm_state`, `position_error`), and every controller holds a safe default until told otherwise.

#### telemetry_node
- Aggregates mode, health heartbeats, and key signals into a low-rate telemetry stream for the host. 
- Completely downstream of control path

#### host_link
- Abstracts the mothership interface.
- Handles communication protocols

### A.3 Control Hierarchy

Two loops share one job:

- **Coarse loop** (camera → coarse_controller → gimbal): wide range, ~5 Hz bandwidth. Points and acquires.
- **Fine loop** (IMU + camera → estimator → FSM): ±1 mrad range, 1 kHz. Holds lock and rejects vibration.

**Acquisition.** The gimbal slews to the commanded bearing (from orbit knowledge / host) and holds. Bearing knowledge is assumed accurate to within the camera field of view (±5 mrad), so the spot lands on the detector without a search. On a valid spot, mode_manager commands HANDOFF.

**Handoff.** The fine loop closes on the estimated error while the gimbal holds position. HANDOFF exits to LOCK when the error stays below 50 µrad for 200 ms. If that isn't met within 2000 ms, HANDOFF aborts back to ACQUIRE. Handoff is a real state with entry, exit, and abort criteria because the transient where the FSM first grabs the beam is where the failures live.

**Lock and offload.** In LOCK the FSM owns the position error entirely. The gimbal ignores the camera and tracks a low-passed version (τ ≈ 1 s) of the FSM's deflection: it steers toward wherever the FSM is straining, and the FSM re-centers itself. Saturation is managed continuously instead of as an event; deflection > 90 % of range raises a health flag.

**Consideration**: both loops consuming camera error simultaneously will fight unless carefully frequency-separated. Making the gimbal jump when the FSM nears its limit can turn FSM saturation into a sudden jolt that can break the lock, which may work, but requires a lot of tuning. With offload, one error signal has only one owner at a time, and the gimbal will move smoothly instead of jumping.

**Loss of lock.** When the camera stops reporting a valid spot, the response is tiered:

1. **COAST** — hold on IMU dead-reckoning for up to 500 ms (30 missed frames); brief dropouts usually self-heal, and re-acquiring over a one-frame gap would cost seconds of link.
2. If the spot returns, re-enter LOCK through the same debounce as HANDOFF.
3. Otherwise fall back to ACQUIRE — re-slew to the last known bearing.

### A.4 Timing and latency budget

### A.5 ROS2 middleware choices

### A.6 Failure Handling

## Part B: The hardest part

## Part D: Design Considerations
