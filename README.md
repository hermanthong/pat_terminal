# PAT Terminal Design Document

This repo models the on-board software for a laser communications Pointing, Acquisition and Tracking (PAT)
terminal. The terminal has to acquire a counterpart terminal, lock onto it, and hold a tight pointing lock while both platforms move and vibrate.

### Deployment
This repo ships a Dockerfile. Uses ROS2 Humble.

### Glossary
- **Spot** is the beacon's image on the detector.
- **Position error** is the spot's offset from the detector center.
- **Handoff** is the transfer of pointing authority from the coarse loop to the fine loop.
- **Mode** is the terminal's operating regime (IDLE, ACQUIRE, HANDOFF, LOCK, COAST, SAFE). Modes are the states of mode_manager's state machine, and thus the overall state of the PAT terminal. Elsewhere, "state" (as in `fsm_state`) means an actuator's physical state. Note thate modes are always written in CAPS LOCK.
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

> [!NOTE]
> **Assumption**: These are assumed numbers from the description of the design challenge. All of them are parameters in the implementation.

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

> [!NOTE]
> exposure time should be used over publish time for accuracy's sake

> [!NOTE]
> **Assumption**: The two axes (azimuth/elevation) are decoupled and the camera is calibrated, so it reports angular error directly in the FSM's axes. On top of that, there's no rolling shutter artifacts, motion blur, nor distortion.

#### imu_node
- IMU driver
- Handles calibration transformations
- Publishes calibrated angular rates and attitude at 1 kHz

> [!NOTE]
> There should not be any filters here. It is the estimator's job to calculate the beam's pointing error at any given time stamp, which is fused from the IMU + camera data.

#### fine_controller
- Owns the FSM
- Owns the pointing-error estimate
- Publishes its own deflection state for the offload loop and for mode supervision

- Estimator that outputs where is the beam pointing right now
  - Complementary filter ([reference](https://www.olliw.eu/2013/imu-data-fusing/)): $\theta_k = \alpha (\theta_{k-1} + \omega_k \Delta t) + (1-\alpha) a_k$, where $\theta$ is the estimated position error, $\omega_k$ the IMU rate and $a_k$ the camera measurement.
  - Each cycle is a weighted average of two opinions: the IMU-propagated prediction with weight α ≈ 0.95, and the camera measurement with weight 1−α. The blend only runs on cycles where a fresh valid frame arrived (60 Hz); propagation runs every 1 ms.
  - The two sensors fail in opposite frequency bands. There is IMU drift, and the camera cannot see vibration. this is the minimal fusion that exploits that
  - A missing or invalid camera frame is simply α = 1 for that cycle, meaning the estimate coasts on IMU propagation.
  - O(1) fixed-cost arithmetic per cycle. fit for the high frequency 1 kHz path

> [!NOTE]
> **Alternative considered**: a latency-compensated estimator could be implemented instead by buffering IMU history and applying each camera correction at its measurement time. It is not needed at these numbers. With a small camera weight (1−α ≈ 0.05 per frame at 60 fps) the camera branch only has authority below f_c ≈ (1−α)·60/2π ≈ 0.5 Hz. Everything faster is corrected by the IMU at < 1 ms latency. Applying a 0.5 Hz signal 30 ms late gives a phase error of 2π·0.5·0.03 ≈ 0.1 rad, so the correction mis-aims by ~10% of the sub-Hz residual (~10 µrad in LOCK). I assume this is acceptable in this context.

#### coarse_controller
- Owns the gimbal
- Slews to the commanded bearing
- Offload
  
#### mode_manager
  - Owns the state machine and is the single writer of `mode`
  - Exposes `set_mode` for the host to command
  - Modes:
    - **IDLE**: powered and healthy, doing nothing. FSM centered, gimbal halted
    - **ACQUIRE**: coarse control only owns position error. gimbal slews to the commanded bearing and holds, waiting for a valid spot
    - **HANDOFF**: fine loop closes on the spot while the gimbal holds. Exits to LOCK on debounced low error, aborts back to ACQUIRE on timeout
    - **LOCK**: fine control only owns position error.
    - **COAST**: camera can't find the spot. Use IMU to keep still, then re-enter LOCK or fall back to ACQUIRE
    - **SAFE**: FSM centered, gimbal halted, waiting for the host. reachable from every state

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> ACQUIRE : set_mode
    ACQUIRE --> HANDOFF : valid spot
    HANDOFF --> LOCK : err < 50 µrad for 200 ms
    HANDOFF --> ACQUIRE : 2000 ms timeout
    LOCK --> COAST : no valid spot 100 ms
    COAST --> LOCK : spot returns (debounced)
    COAST --> ACQUIRE : 500 ms timeout
    IDLE --> SAFE
    ACQUIRE --> SAFE
    HANDOFF --> SAFE
    LOCK --> SAFE
    COAST --> SAFE
    SAFE --> IDLE : set_mode
```

> [!NOTE]
> `set_mode` is a service because the host must know if the command is accepted or rejected.

> [!NOTE]
> `mode` is distributed as a topic, not many per-node services. It is state that many nodes need, including ones that weren't alive when it changed (reliable + transient-local delivers the current mode to late joiners). Instead, mode_manager supervises behavior (`fsm_state`, `position_error`), and every controller holds a safe default until told otherwise.

#### telemetry_node
- Aggregates mode, health heartbeats, and key signals into a low-rate telemetry stream for the host. 
- Completely downstream of control path

#### host_link
- Abstracts the mothership interface.
- Handles communication protocols

> [!NOTE]
> **Assumption**: The host is abstracted to the `set_mode` service and the telemetry topics. Everything else behind host_link is out of scope.

### A.3 Control Hierarchy

Two loops share one job:

- **Coarse loop** (camera → coarse_controller → gimbal): wide range, ~5 Hz bandwidth. Points and acquires.
- **Fine loop** (IMU + camera → estimator → FSM): ±1 mrad range, 1 kHz. Holds lock and rejects vibration.

> [!NOTE]
> **Assumption**: The counterpart's beacon is always on. There's no atmospheric model, and all disturbance is platform-side: vibration plus slow relative drift.

**Acquisition.** The gimbal slews to the commanded bearing (from orbit knowledge / host) and holds. On a valid spot, mode_manager commands HANDOFF.

> [!NOTE]
> **Assumption**: Bearing knowledge from the host is accurate to within the camera field of view, so the spot always lands on the detector without the need of a search scan.

**Handoff.** The fine loop closes on the estimated error while the gimbal holds position. HANDOFF exits to LOCK when the error stays below 50 µrad for 200 ms. If that isn't met within 2000 ms, HANDOFF aborts back to ACQUIRE. Handoff is a real state with entry, exit, and abort criteria because the transient where the FSM first grabs the beam is where the failures live.

**Lock and offload.** In LOCK the FSM owns the position error entirely. The gimbal ignores the camera and tracks a low-passed version (τ ≈ 1 s) of the FSM's deflection: it steers toward wherever the FSM is straining, and the FSM re-centers itself. Saturation is managed continuously instead of as an event.

> [!NOTE]
> **Alternative considered**: Both loops consuming camera error simultaneously will fight unless carefully frequency-separated. Making the gimbal jump when the FSM nears its limit can turn FSM saturation into a sudden jolt that can break the lock, which may work, but requires a lot of tuning. With offload, one error signal has only one owner at a time, and the gimbal will move smoothly instead of jumping.

#### Loss of lock
When the camera stops reporting a valid spot, the response is tiered:

1. **COAST**: hold on IMU dead-reckoning for up to 500 ms. Brief dropouts usually self-heal, and re-acquiring over a one-frame gap would cost seconds of link.
2. If the spot returns, re-enter LOCK through the same debounce as HANDOFF.
3. Otherwise fall back to ACQUIRE. Re-slew to the last known bearing.

### A.4 Timing and latency budget

### A.5 ROS2 middleware choices
#### QoS Profiles

| Topic / service | Reliability | Durability | History | Reason |
|---|---|---|---|---|
| `/imu` | best effort | volatile | keep last 1 | The most recent sensor data is the most important |
| `/position_error` | best effort | volatile | keep last 1 | The most recent sensor data is the most important. validity flag + stamp let downstream nodes reason about gaps |
| `/fsm_cmd`, `/gimbal_cmd` | best effort | volatile | keep last 1 | Only the last command matters |
| `/fsm_state`, `/gimbal_state` | best effort | volatile | keep last 1 | Only the last command matters |
| `/mode` | reliable | transient local | keep last 1 | A late-joining node must immediately learn the current mode. Missing a mode change is not acceptable |
| `/telemetry`, `/health` | reliable | volatile | keep last 10 | This data is not time-critical. Robustness of logging is more important |
| `/set_mode` (service) | reliable (service default) | — | — | Host needs to know if their command was accepted or rejected |

#### Executors and callback groups
The fine_controller process is where starvation would hurt, so it is structured explicitly:

- A **real-time callback group** (mutually exclusive):
  - 1 kHz control timer
  - IMU subscription.
  - single-threaded executor on a dedicated thread.
- A **housekeeping group** for everything else (default group):
  - camera correction
  - mode subscription
  - state publishing
  - parameter callbacks
  - etc.
  
> [!NOTE]
> ROS2 executors have no priority setting. Under load, using a multithreaded executor with everything in the same callback group may cause lower priority tasks like parameter callbacks to delay the control loop.

> [!NOTE]
> The real-time callback group is mutually exclusive because it guarantees that the timer + IMU callbacks never overlap, so they can share the estimator state without having to manually handle a mutex. The housekeeping group doesn't really matter because it is not time-critical.

#### Discovery and late join
DDS discovery means nodes can start in any order and links form whenever both ends exist.
- `/mode` is transient-local, so any node that starts late (or restarts) receives
  the current mode immediately on subscription match.
- Until a first mode message arrives, every controller is in the SAFE mode. A controller that boots mid-LOCK therefore behaves as if in SAFE until told otherwise, and mode_manager's supervision reacts to the resulting loss of lock as it would to any other.
- Service clients use `wait_for_service` with timeout rather than assuming the server exists.

> [!NOTE]
> ROS2 lifecycle nodes with launch orchestration were considered: bring every node to `active` before any mode leaves IDLE, and no one joins late. However, that only solves the startup case. A node that crashes and restarts mid-run, or a subscription that matches late in a DDS discovery race may still break the system. A transient-local `/mode` plus safe defaults covers startup as well as error recovery. Note that lifecycle nodes could still be added for orderly startup if it helps.

### A.6 Failure Handling
| Failure | Detection | Response |
|---|---|---|
| Lost camera frames | inter-frame watchdog | mode_manager enters COAST after 100 ms without a valid spot. The estimator dead-reckons on the IMU. After 500 ms without the spot returning, mode_manager falls back to ACQUIRE. |
| IMU invalid data | rate-of-change and range gating on each sample | Bad samples are dropped and the previous rate is held. IMU silence > 5 ms in LOCK means the spec is unmeetable, so mode_manager enters SAFE. |
| Mode transition safety | — | Entry actions run on every transition: ACQUIRE centers the FSM, SAFE centers the FSM and halts the gimbal. |
| Node crash / silence | stale-data watchdogs on control topics | Controllers fall back to safe defaults on stale inputs. mode_manager enters SAFE instead of acting on stale state. The node is relaunched (launch-file respawn), learns the current mode from transient-local `/mode`, and operation resumes. |


---

## Part B: The hardest part
Ranked hardest first:

1. **The coarse/fine handoff and its failure modes.** System integration is always hard.
2. **Fusing a delayed, slow measurement with a fast sensor.** The camera's 30 ms latency against a 1 kHz loop.
3. **Holding hard real-time in ROS 2.** Executor isolation, QoS discipline, RT kernel configuration. This requires careful programming and identifying chokepoints.

> [!NOTE]
> Items 2 and 3 are hard, but at the end of the day, they are one component with one job and clear success criteria. On top of that, there are known solutions for them, it just requires tuning.
> 
> The handoff is hard because it is an interaction between systems. The two controllers have opposite dynamics, a state machine, and imperfect sensing. Every part can be individually correct but the system still fails because of emergent properties. The failure modes are interaction failures. For example, the FSM could slamming into its range limit because the gimbal hadn't settled, offload and re-acquisition could fight over the gimbal, or a stale mode message could leave two controllers each believing they own the beam. It is impossible to write unit tests for these parts.

In order to build these, the following packages are required:
1. pat_interfaces: definition of messages and services
1. plant_sim: simulation of sensors and actuators. not an actual part of the PAT terminal.
1. fine_controller: 1 kHz loop
1. coarse_controller: 60 Hz loop
1. mode_manager: overall state machine

> [!NOTE]
> **Assumption**: One terminal is modeled. The counterpart is a fixed, slowly drifting bearing in the sim.

> [!NOTE]
> **Alternative considered**: simulated-clock testing (`use_sim_time`) for deterministic, faster-than-real-time tests. However, I will just implement the tests running with the wall clock due to time constraints.


---

## Part D: Design Considerations
