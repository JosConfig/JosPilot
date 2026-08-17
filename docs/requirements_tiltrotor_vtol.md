# Requirements Specification: Tiltrotor Tricopter VTOL (ArduPlane / QuadPlane)

**Document Number:** TILT-SyRS-001
**Version:** 1.0
**Status:** Draft
**Classification:** Internal
**Standard Compliance:** ENG-STD-REQS-001 v2.0 (ISO/IEC/IEEE 29148, DO-178C, ARP4754A, INCOSE V5, ECSS-E-ST-10-06C)

---

## Document Version History

| Version | Date | Author | Description |
|---|---|---|---|
| 1.0 | 2025 | Systems Engineering | Initial release. Derived from ArduPlane source code analysis. |

---

## 1. Introduction

This document specifies the system requirements for the Tiltrotor Tricopter VTOL
configuration implemented within the ArduPlane QuadPlane framework. Requirements are
derived from source code analysis of `ArduPlane/tiltrotor.h`, `ArduPlane/tiltrotor.cpp`,
`ArduPlane/quadplane.h`, `ArduPlane/quadplane.cpp`, `ArduPlane/transition.h`,
`libraries/AP_Motors/AP_MotorsTri.h`, and `libraries/AP_Motors/AP_MotorsTri.cpp`.

All requirements in this document are authored in compliance with ENG-STD-REQS-001 v2.0.

---

## 2. Definitions and Conventions

- **shall** — mandatory binding obligation (ISO/IEC/IEEE 29148).
- **should** — non-mandatory recommendation.
- **will** — statement of fact or declared intent.
- **may** — permitted but not required.
- **must** — PROHIBITED in this document. All obligations use **shall**.
- **VTOL** — Vertical Take-Off and Landing.
- **FW** — Fixed-Wing.
- **TECS** — Total Energy Control System (ArduPlane throttle/pitch controller).
- **GCS** — Ground Control Station.

---

## 3. Attribute Legend

Each requirement carries the following mandatory attributes:

| Attribute | Description |
|---|---|
| **ID** | Unique identifier. Format: TILT-SyRS-NNNN |
| **Description** | The full requirement statement (SOPHIST template). |
| **Source** | Origin: ArduPlane source file, section, or standard clause. |
| **Rationale** | Why this requirement is needed. |
| **Stakeholder** | Role(s) responsible for this requirement. |
| **Type** | Functional / Safety & Reliability / Performance / Design Constraint / Interface |
| **Revision** | 001 (initial). |
| **Verification Method** | T=Test, I=Inspection, A=Analysis, D=Demonstration. |
| **Parent** | Parent requirement ID, or NONE for top-level. |

---

## 4. Hardware and Frame Requirements

### 4.1 Motor Layout

---

**TILT-SyRS-0001**
The tiltrotor system shall accept a minimum of two active VTOL motors during hover operation.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::init()` — MOT1, MOT2, MOT4 registration.
- **Rationale:** A minimum of two motors is required to provide roll and pitch authority in hover. The tricopter variant uses three outputs (MOT1, MOT2, MOT4).
- **Stakeholder:** Avionics Systems Engineer; Vehicle Integration Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Activate tiltrotor firmware on a tricopter frame; confirm motor arming and spin-up on at least two motor outputs.
- **Parent:** NONE.

---

**TILT-SyRS-0002**
The tiltrotor system shall use the parameter `Q_TILT_MASK` to identify, as a bitmask, the set of motors that are capable of tilting, where bit N corresponds to motor N+1.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::var_info[]` — parameter index 2; `tiltrotor.cpp::is_motor_tilting()`.
- **Rationale:** The bitmask allows flexible assignment of tilt capability to any subset of motors without firmware changes.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect parameter definition and `is_motor_tilting()` logic for correct bitmask evaluation.
- **Parent:** TILT-SyRS-0001.

---

**TILT-SyRS-0003**
When `Q_TILT_MASK` is set to zero, the tiltrotor system shall disable all tiltrotor-specific control outputs and revert to standard QuadPlane motor behaviour.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::update()` — guard `if (!enabled() || tilt_mask == 0) return`.
- **Rationale:** A zero mask indicates no tilting motors are present; executing tilt logic would produce undefined output.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_TILT_MASK=0`; confirm no tilt servo output is generated.
- **Parent:** TILT-SyRS-0002.

---

**TILT-SyRS-0004**
At startup, the tiltrotor system shall detect the presence of a fixed-wing forward thrust motor by checking whether servo function `k_throttle`, `k_throttleLeft`, or `k_throttleRight` is assigned, and shall record the result in the internal `_have_fw_motor` flag.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::setup()` — `_have_fw_motor` assignment block.
- **Rationale:** Knowing whether a dedicated forward motor exists alters transition throttle blending and TECS handoff logic.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect `setup()` for correct channel detection logic covering all three servo functions.
- **Parent:** TILT-SyRS-0001.

---

**TILT-SyRS-0005**
At startup, the tiltrotor system shall detect the presence of at least one permanently upward-facing (non-tilting) VTOL motor and shall record the result in the internal `_have_vtol_motor` flag.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::setup()` — `_have_vtol_motor` loop over `AP_MOTORS_MAX_NUM_MOTORS`.
- **Rationale:** The presence or absence of a fixed VTOL motor determines which transition throttle blending strategy is applied.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect `setup()` loop for correct exclusion of motors in `tilt_mask`.
- **Parent:** TILT-SyRS-0001.

---

### 4.2 Servo Outputs

---

**TILT-SyRS-0006**
The tiltrotor system shall output the primary tilt position command to servo function `k_motor_tilt` on a scale of 0 to 1000.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::slew()` — `SRV_Channels::set_output_scaled(k_motor_tilt, 1000 * current_tilt)`.
- **Rationale:** A single named servo function decouples the tilt command from physical channel assignment and enables hardware-independent configuration.
- **Stakeholder:** Avionics Systems Engineer; Integration Engineer.
- **Type:** Interface.
- **Revision:** 001.
- **Verification Method:** T — Command tilt from 0 to 1.0; measure `k_motor_tilt` output ranging from 0 to 1000.
- **Parent:** TILT-SyRS-0002.

---

**TILT-SyRS-0007**
When `Q_TILT_TYPE` is set to 2 (Vectored Yaw), the tiltrotor system shall output independent tilt position commands to each of the five servo functions: `k_tiltMotorLeft`, `k_tiltMotorRight`, `k_tiltMotorRear`, `k_tiltMotorRearLeft`, and `k_tiltMotorRearRight`, each on a scale of 0 to 1000.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::setup()` — range set to 1000 for all five channels; `vectoring()` — individual channel writes.
- **Rationale:** Five independent servo outputs are required to allow differential deflection for yaw vectoring and to support asymmetric motor layouts.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Interface.
- **Revision:** 001.
- **Verification Method:** T — With `Q_TILT_TYPE=2`, apply yaw demand; confirm differential outputs on left and right tilt channels while rear channel tracks base tilt.
- **Parent:** TILT-SyRS-0006.

---

**TILT-SyRS-0008**
When configured as a tricopter (`Q_FRAME_CLASS=7`), the tiltrotor system shall assign the yaw servo output to channel 7 (`k_motor7`) and shall constrain its commanded angle to the range 5° to 80°.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.h` — `AP_MOTORS_CH_TRI_YAW = CH_7`; `AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN=5`, `AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX=80`.
- **Rationale:** Physical servo travel limits define the achievable yaw pivot range; commanding outside this range would saturate the servo and reduce control authority.
- **Stakeholder:** Avionics Systems Engineer; Mechanical Integration Engineer.
- **Type:** Performance.
- **Revision:** 001.
- **Verification Method:** T — Command maximum yaw demand; confirm servo output is clamped at 80° and does not exceed 80°.
- **Parent:** TILT-SyRS-0006.

---

**TILT-SyRS-0009**
When `Q_TILT_TYPE` is set to 3 (Bicopter), the tiltrotor system shall output tilt commands via servo functions `k_tiltMotorLeft` and `k_tiltMotorRight` only.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::bicopter_output()`.
- **Rationale:** The bicopter layout uses exactly two tilting motors; no other tilt servo functions are valid for this type.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Interface.
- **Revision:** 001.
- **Verification Method:** I — Inspect `bicopter_output()` for exclusive use of `k_tiltMotorLeft` and `k_tiltMotorRight`.
- **Parent:** TILT-SyRS-0006.

---

**TILT-SyRS-0010**
When `Q_TILT_TYPE` is set to 3 (Bicopter), the tiltrotor system shall require `Q_FRAME_CLASS` to be set to 10 (Tailsitter) for correct motor mixing initialisation.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::setup()` — `TILT_TYPE_BICOPTER` check excludes `k_throttleLeft`/`k_throttleRight` from `_have_fw_motor`.
- **Rationale:** The bicopter variant uses the Tailsitter motor class for its control mix; using any other frame class produces incorrect thrust allocation.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Design Constraint.
- **Revision:** 001.
- **Verification Method:** I — Inspect `setup()` for `TILT_TYPE_BICOPTER` guard on frame class requirement.
- **Parent:** TILT-SyRS-0009.

---

**TILT-SyRS-0011**
The tiltrotor system shall report its MAVLink vehicle type as `MAV_TYPE_VTOL_TILTROTOR` (value 21) in HEARTBEAT messages.

- **Source:** `ArduPlane/quadplane.cpp::QuadPlane::var_info[]` — `MAV_TYPE` parameter index 57, value 21.
- **Rationale:** Ground control stations use the MAVLink vehicle type to select appropriate displays and command sets. An incorrect type causes GCS to apply wrong control mappings.
- **Stakeholder:** Ground Segment Engineer; GCS Integration Engineer.
- **Type:** Interface.
- **Revision:** 001.
- **Verification Method:** T — Monitor MAVLink HEARTBEAT messages; confirm `type` field equals 21.
- **Parent:** NONE.

---

## 5. Tilt Mechanism Types

---

**TILT-SyRS-0012**
The tiltrotor system shall support four tilt mechanism types, selectable via parameter `Q_TILT_TYPE`: 0 (Continuous), 1 (Binary), 2 (Vectored Yaw), and 3 (Bicopter).

- **Source:** `ArduPlane/tiltrotor.h` — enum `TILT_TYPE_CONTINUOUS=0`, `TILT_TYPE_BINARY=1`, `TILT_TYPE_VECTORED_YAW=2`, `TILT_TYPE_BICOPTER=3`.
- **Rationale:** Different airframe designs require different tilt servo actuation mechanisms; a single configurable parameter allows the same firmware to support all variants.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect `update()` dispatch logic for all four type values.
- **Parent:** TILT-SyRS-0002.

---

**TILT-SyRS-0013**
When `Q_TILT_TYPE` is set to 0 (Continuous), the tiltrotor system shall move tilt servos continuously between 0° (hover) and 90° (forward flight) at a rate limited by `Q_TILT_RATE_UP` when tilting toward hover and by `Q_TILT_RATE_DN` when tilting toward forward flight.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()`; `slew()`; `tilt_max_change()`.
- **Rationale:** Continuous slewing allows precise intermediate tilt angles, enabling the system to use tilt position as a forward thrust control input during VTOL flight.
- **Stakeholder:** Avionics Systems Engineer; Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Command tilt slew at maximum rate; measure servo travel rate against `Q_TILT_RATE_UP` and `Q_TILT_RATE_DN` values respectively.
- **Parent:** TILT-SyRS-0012.

---

**TILT-SyRS-0014**
When `Q_TILT_TYPE` is set to 1 (Binary), the tiltrotor system shall command the tilt servo output to either fully forward (1000) or fully up (0) with no intermediate positions, while maintaining a rate-limited internal `current_tilt` tracking value to govern throttle handoff timing.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::binary_slew()`; `binary_update()`.
- **Rationale:** Retract-style servos only support two positions; the rate-limited `current_tilt` tracker is still needed to sequence the transition throttle handoff correctly.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — With `Q_TILT_TYPE=1`, command VTOL-to-FW transition; confirm servo jumps to 1000 while `current_tilt` increments at the configured rate.
- **Parent:** TILT-SyRS-0012.

---

**TILT-SyRS-0015**
When `Q_TILT_TYPE` is set to 2 (Vectored Yaw), the tiltrotor system shall apply differential deflection to the left and right tilt servos, centred on the base tilt angle, to produce yaw control torque in hover proportional to the yaw demand and scaled by `Q_TILT_YAW_ANGLE`.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::vectoring()` — `tilt_offset = tilt_scale * yaw_range`.
- **Rationale:** Vectored yaw replaces or supplements differential motor torque for yaw authority, which is essential for airframes without a dedicated yaw servo.
- **Stakeholder:** Avionics Systems Engineer; Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Apply maximum yaw demand in hover with `Q_TILT_TYPE=2`; confirm left and right tilt outputs diverge symmetrically around the base tilt position.
- **Parent:** TILT-SyRS-0012.

---

**TILT-SyRS-0016**
When `Q_TILT_TYPE` is set to 3 (Bicopter), the tiltrotor system shall scale tilt servo authority by `cos(current_tilt × π/2)` as the motors tilt forward, reducing control authority to zero at full forward tilt.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::bicopter_output()` — `scaling = cosf(current_tilt * M_PI_2)`.
- **Rationale:** As bicopter motors tilt toward horizontal, their vertical thrust component and attitude control authority both reduce by the cosine of the tilt angle; scaling prevents over-commanding.
- **Stakeholder:** Avionics Systems Engineer; Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify via analysis that at `current_tilt=1.0` the scaling factor equals `cos(π/2) = 0`, and at `current_tilt=0` the factor equals 1.0.
- **Parent:** TILT-SyRS-0012.

---

## 6. Tilt Control Parameters

The following table defines all parameters in the `Q_TILT_` group. Each parameter constitutes a Design Constraint requirement on the parameter storage subsystem.

| Parameter | AP_Param Index | Default | Valid Range | Units | Description |
|---|---|---|---|---|---|
| `Q_TILT_ENABLE` | 1 | 0 | 0, 1 | — | Enables tiltrotor functionality. Auto-set to 1 if `Q_TILT_MASK` is non-zero or `Q_TILT_TYPE=3`. Requires reboot. |
| `Q_TILT_MASK` | 2 | 0 | 0–4095 (bitmask) | — | Bitmask of tiltable motors. Bit N enables motor N+1. |
| `Q_TILT_RATE_UP` | 3 | 40 | 10–300 | deg/s | Maximum tilt rate toward hover (tilt decreasing). |
| `Q_TILT_MAX` | 4 | 45 | 20–80 | deg | Maximum tilt angle at which VTOL motor control is active. Beyond this angle the vehicle operates as a fixed-wing aircraft. |
| `Q_TILT_TYPE` | 5 | 0 | 0–3 | — | Tilt mechanism type. 0=Continuous, 1=Binary, 2=VectoredYaw, 3=Bicopter. |
| `Q_TILT_RATE_DN` | 6 | 0 | 10–300 | deg/s | Maximum tilt rate toward forward flight (tilt increasing). When 0, `Q_TILT_RATE_UP` is used for both directions. |
| `Q_TILT_YAW_ANGLE` | 7 | 0 | 0–30 | deg | Baseline servo angle from vertical for vectored-yaw authority. Also limits maximum forward tilt in Bicopter mode. |
| `Q_TILT_FIX_ANGLE` | 8 | 0 | 0–30 | deg | Maximum downward tilt angle at full throttle in fixed-wing mode, enabling roll and pitch vectoring. |
| `Q_TILT_FIX_GAIN` | 9 | 0 | 0–1 | — | Gain applied to tilt vectoring output in fixed-wing flight. |
| `Q_TILT_WING_FLAP` | 10 | 0 | 0–15 | deg | Wing tilt angle used as flap for tilt-wing airframes. A value greater than 0 defines the partial-tilt forward flight position. |

---

**TILT-SyRS-0017**
The tiltrotor system shall provide parameter `Q_TILT_ENABLE` at `AP_Param` group index 1, with a default value of 0, and shall require a system reboot for changes to take effect.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::var_info[]` index 1 — `AP_PARAM_FLAG_ENABLE`.
- **Rationale:** The enable flag controls allocation of motor mixing objects at boot; runtime toggling without reboot would leave allocated objects in an inconsistent state.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Design Constraint.
- **Revision:** 001.
- **Verification Method:** I — Inspect parameter declaration for `AP_PARAM_FLAG_ENABLE` and `RebootRequired` annotation.
- **Parent:** TILT-SyRS-0012.

---

**TILT-SyRS-0018**
When `Q_TILT_MASK` is non-zero at startup and `Q_TILT_ENABLE` has not been explicitly configured, the tiltrotor system shall automatically set `Q_TILT_ENABLE` to 1 and save the value to non-volatile storage.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::setup()` — `if (!enable.configured() && (tilt_mask != 0 || type == TILT_TYPE_BICOPTER)) enable.set_and_save(1)`.
- **Rationale:** Prevents a configuration failure mode where a user sets a tilt mask but forgets to enable the feature, which would silently produce no tilt output.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_TILT_MASK` to non-zero without setting `Q_TILT_ENABLE`; reboot; confirm `Q_TILT_ENABLE` reads as 1.
- **Parent:** TILT-SyRS-0017.

---

**TILT-SyRS-0019**
The tiltrotor system shall accept values for `Q_TILT_RATE_UP` in the range 10 to 300 degrees per second, with a default of 40 degrees per second.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::var_info[]` index 3.
- **Rationale:** Values below 10 deg/s would make transitions impractically slow; values above 300 deg/s exceed typical servo capability and risk structural loads.
- **Stakeholder:** Avionics Systems Engineer; Flight Control Engineer.
- **Type:** Performance.
- **Revision:** 001.
- **Verification Method:** I — Inspect parameter range declaration for `max_rate_up_dps`.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0020**
The tiltrotor system shall accept values for `Q_TILT_MAX` in the range 20 to 80 degrees, with a default of 45 degrees.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::var_info[]` index 4.
- **Rationale:** Below 20°, insufficient forward thrust is available for transition; above 80°, the motor approaches horizontal and VTOL attitude authority is lost before transition is complete.
- **Stakeholder:** Avionics Systems Engineer; Flight Control Engineer.
- **Type:** Performance.
- **Revision:** 001.
- **Verification Method:** I — Inspect parameter range declaration for `max_angle_deg`.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0021**
When `Q_TILT_WING_FLAP` is greater than 0 degrees, the tiltrotor system shall define the fully-forward tilt position as `1.0 − (Q_TILT_WING_FLAP / 90.0)`, allowing the wing to remain partially tilted in forward flight to generate lift-increasing flap effect.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::get_fully_forward_tilt()` — `return 1.0 - (flap_angle_deg * (1/90.0))`.
- **Rationale:** Tilt-wing aircraft maintain partial wing tilt in cruise to increase lift coefficient, reducing stall speed. The formula converts the flap angle in degrees to a normalised tilt value.
- **Stakeholder:** Avionics Systems Engineer; Aerodynamics Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify analytically that `get_fully_forward_tilt()` returns `1.0 - (flap_angle_deg / 90.0)` for representative values.
- **Parent:** TILT-SyRS-0013.

---

## 7. Tilt Rate and Slew Control

---

**TILT-SyRS-0022**
The tiltrotor system shall limit the rate of change of the tilt position to `Q_TILT_RATE_UP` degrees per second when tilting toward the hover position (decreasing `current_tilt`).

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_max_change()` — `rate = max_rate_up_dps` when `up=true`.
- **Rationale:** An uncontrolled tilt-up at maximum servo speed causes an abrupt loss of forward thrust and a sudden increase in VTOL rotor load, risking motor overload and vehicle upset.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Command full tilt-up transition; measure `current_tilt` rate-of-change and confirm it does not exceed `Q_TILT_RATE_UP` deg/s.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0023**
The tiltrotor system shall limit the rate of change of the tilt position to `Q_TILT_RATE_DN` degrees per second when tilting toward the forward flight position (increasing `current_tilt`), and shall use `Q_TILT_RATE_UP` as the rate limit when `Q_TILT_RATE_DN` is set to 0.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_max_change()` — `if (up || max_rate_down_dps <= 0) rate = max_rate_up_dps; else rate = max_rate_down_dps`.
- **Rationale:** A separate down-tilt rate allows operators to configure a faster tilt-to-FW than tilt-to-hover. Setting `Q_TILT_RATE_DN=0` reuses the hover rate to simplify single-rate configurations.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_TILT_RATE_DN` to 100 deg/s; command tilt-down; confirm rate does not exceed 100 deg/s. Repeat with `Q_TILT_RATE_DN=0` and confirm rate matches `Q_TILT_RATE_UP`.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0024**
When the vehicle is in MANUAL mode, the tiltrotor system shall permit a tilt slew rate of at least 90 degrees per second regardless of the value of `Q_TILT_RATE_DN`.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_max_change()` — `if (fast_tilt) rate = MAX(rate, 90)`.
- **Rationale:** MANUAL mode indicates a test or recovery context where rapid servo response is required; capping below 90 deg/s in manual would impair ground testing and emergency response.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — With `Q_TILT_RATE_DN=10`, enter MANUAL mode and command tilt-forward; confirm servo reaches full forward within 1 second (i.e., rate ≥ 90 deg/s).
- **Parent:** TILT-SyRS-0022.

---

**TILT-SyRS-0025**
When the vehicle is armed and not in a VTOL mode and not in assisted flight, the tiltrotor system shall permit a tilt slew rate of at least 90 degrees per second regardless of the value of `Q_TILT_RATE_DN`.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_max_change()` — `if (arming.is_armed_and_safety_off() && !in_vtol_mode() && !assisted_flight) fast_tilt = true`.
- **Rationale:** In armed fixed-wing flight, the vehicle must be able to tilt rotors forward rapidly to maintain forward thrust without delay.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Arm vehicle in a fixed-wing mode without VTOL assist; command tilt-forward; confirm slew rate is not below 90 deg/s.
- **Parent:** TILT-SyRS-0022.

---

**TILT-SyRS-0026**
The tiltrotor system shall set the `angle_achieved` flag to true when `current_tilt` equals the commanded target tilt value, and shall set it to false whenever a slew-rate limit prevents the current tilt from reaching the target within a single control cycle.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::slew()` — `angle_achieved = is_equal(newtilt, current_tilt)`.
- **Rationale:** The transition state machine uses `tilt_angle_achieved()` to gate the DONE state; an incorrect flag would cause premature or delayed transition completion.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect `slew()` for correct `is_equal()` comparison and flag assignment.
- **Parent:** TILT-SyRS-0013.

---

## 8. Tilt Compensation (Thrust Mixing)

---

**TILT-SyRS-0027**
When in VTOL mode and tilt angle is greater than 0°, the tiltrotor system shall multiply the thrust command of each non-tilting motor by `cos(current_tilt × 90°)` to compensate for the reduction in vertical thrust contribution as tilting motors rotate forward.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_compensate()` — `tilt_factor = cosf(radians(current_tilt*90)); tilt_compensate_angle(thrust, num_motors, tilt_factor, 1)`.
- **Rationale:** As tilting motors rotate forward, they contribute less vertical force. Non-tilting motors must increase output proportionally to maintain total vertical thrust and prevent altitude loss.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify analytically that the compensation factor equals 1.0 at tilt=0 and approaches 0 at tilt=1.0 (90°).
- **Parent:** TILT-SyRS-0002.

---

**TILT-SyRS-0028**
When in fixed-wing mode and tilt angle is greater than 0° and less than or equal to 0.98 (88.2°), the tiltrotor system shall multiply the thrust command of each tilting motor by `1 / cos(current_tilt × 90°)` to compensate for the angular inefficiency of partially tilted rotors producing forward thrust.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_compensate()` — `inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90))` with `current_tilt` capped at 0.98.
- **Rationale:** The tilt cap at 0.98 prevents division by cos(≈90°) approaching zero, which would produce infinite gain and motor output saturation.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify that compensation factor is bounded and that for `current_tilt=0.98` the factor equals `1/cos(88.2°) ≈ 33`; confirm this is the maximum applied value.
- **Parent:** TILT-SyRS-0027.

---

**TILT-SyRS-0029**
The tiltrotor system shall blend the roll authority of each tilting motor from its full differential value toward the average tilt thrust by the factor `current_tilt`, so that at full tilt (`current_tilt=1.0`) roll authority from tilting motors is zero.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_compensate_angle()` — `thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1 - current_tilt)`.
- **Rationale:** Tilted motors produce yaw rather than roll when tilted forward; retaining full roll gain at large tilt angles causes control axis coupling and potential roll oscillation.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify that at `current_tilt=0` the roll contribution is unmodified, and at `current_tilt=1.0` it equals the average tilt thrust with no differential roll component.
- **Parent:** TILT-SyRS-0027.

---

**TILT-SyRS-0030**
The tiltrotor system shall add a differential thrust yaw component to each tilting motor's thrust command, proportional to `sin(current_tilt × 90°) × sin(Q_TILT_YAW_ANGLE)`, to maintain consistent yaw control authority across all tilt angles.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_compensate_angle()` — `diff_thrust = get_roll_factor(i) * (get_yaw()+get_yaw_ff()) * sin_tilt * yaw_gain`.
- **Rationale:** Yaw authority from tilted motors increases with tilt angle; scaling by sin(tilt) and sin(yaw_angle) normalises yaw gain so PID tuning remains valid across the tilt range.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify that at tilt=0 the differential yaw term equals zero and at full tilt equals the maximum yaw authority product.
- **Parent:** TILT-SyRS-0027.

---

**TILT-SyRS-0031**
When any tilting motor's compensated thrust command exceeds 1.0 after applying tilt compensation, the tiltrotor system shall scale all motor thrust commands proportionally so that the largest tilting motor command equals 1.0.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::tilt_compensate_angle()` — `if (largest_tilted > 1.0f) { scale = 1.0f / largest_tilted; for(...) thrust[i] *= scale; }`.
- **Rationale:** Motor thrust commands exceeding 1.0 are physically unrealisable; proportional scaling preserves the relative thrust distribution and prevents motor saturation from degrading attitude control.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** A — Verify that after scaling, the maximum thrust command among tilting motors is exactly 1.0 when saturation occurs.
- **Parent:** TILT-SyRS-0027.

---

## 9. Tilt Angle Command Strategy in VTOL Mode (Continuous Type)

The following requirements define the priority-ordered rules for computing the forward tilt angle during VTOL flight. Rules are evaluated in the order listed; the first matching condition applies.

---

**TILT-SyRS-0032**
When the active flight mode is QAutotune, the tiltrotor system shall command tilt to 0° regardless of all other tilt demand inputs.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `if (control_mode == &mode_qautotune) { slew(0); return; }`.
- **Rationale:** Autotune performs aggressive attitude manoeuvres; any forward tilt during autotune would introduce uncontrolled forward acceleration and corrupt PID identification.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Enter QAutotune mode; confirm tilt servo output is 0 throughout the autotune sequence.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0033**
When `Q_FWD_THR_USE` is set to a value greater than 0 and the vehicle is flying in VTOL mode and not in assisted flight, the tiltrotor system shall compute the tilt demand as `min(atan(forward_throttle_pct / 100), Q_TILT_MAX)` degrees.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `fwd_g_demand = 0.01 * forward_throttle_pct(); fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), max_angle_deg)`.
- **Rationale:** The arctangent mapping converts a forward thrust fraction (0–1) to the equivalent motor tilt angle that would produce the same forward force component, providing physically meaningful tilt control.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify that for `forward_throttle_pct=100`, the computed tilt equals `min(atan(1.0) × 180/π, Q_TILT_MAX)` = `min(45°, Q_TILT_MAX)`.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0034**
When the active flight mode is QAcro, QStabilize, or QHover and no forward throttle RC channel is configured, the tiltrotor system shall command tilt to 0°.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `if (rc_fwd_thr_ch == nullptr) { slew(0); }`.
- **Rationale:** Without a forward throttle input, the pilot has no means to command forward flight; maintaining 0° tilt ensures these modes function as safe hover-only recovery modes.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — With no `FWD_THR` RC channel configured, enter QStabilize; confirm tilt output remains at 0 throughout.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0035**
When the active flight mode is QAcro, QStabilize, or QHover and a forward throttle RC channel is configured, the tiltrotor system shall set tilt demand proportionally to the manual forward throttle percentage, up to a maximum of `Q_TILT_MAX` degrees.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `settilt = 0.01f * forward_throttle_pct(); slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt()))`.
- **Rationale:** Manual forward throttle provides the pilot with direct control over forward tilt during hover manoeuvres.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Apply 50% forward throttle RC input in QHover; confirm tilt output equals `0.5 × (Q_TILT_MAX / 90)` normalised units.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0036**
When in assisted flight with transition state at TIMER or beyond, the tiltrotor system shall command tilt to move to the fully-forward position at the configured tilt rate.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `if (assisted_flight && transition_state >= Tiltrotor_Transition::State::TIMER) slew(get_forward_flight_tilt())`.
- **Rationale:** Once the airspeed threshold has been passed and the TIMER transition phase begins, the tilting motors must complete their rotation to full forward to prepare for fixed-wing handoff.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Trigger VTOL-to-FW transition; confirm tilt reaches fully-forward position before transition reaches DONE state.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0037**
In all VTOL flight cases not covered by TILT-SyRS-0032 through TILT-SyRS-0036, the tiltrotor system shall set tilt demand proportionally to throttle output, scaling from 0° at the throttle minimum to `Q_TILT_MAX` at 50% throttle output, and shall not exceed the fully-forward tilt position.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `settilt = constrain_float((throttle - MAX(throttle_min, 0)) * 0.02, 0, 1); slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt()))`.
- **Rationale:** Coupling tilt to throttle in the general case provides automatic forward thrust assist proportional to vertical thrust demand, which improves energy efficiency in cruise-climb situations.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — In QLoiter at 50% throttle output, confirm tilt position reaches `Q_TILT_MAX`; at zero throttle, confirm tilt position is 0.
- **Parent:** TILT-SyRS-0013.

---

## 10. Forward Flight Behaviour

---

**TILT-SyRS-0038**
When the vehicle is not in a VTOL mode and not in assisted flight, the tiltrotor system shall command tilting motors to move to the fully-forward tilt position at the configured tilt rate.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt())` in the fixed-wing branch.
- **Rationale:** In fixed-wing flight, tilting motors must be fully forward to act as forward thrust motors; any partial tilt wastes thrust and creates drag.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Switch from QHover to FBWA; confirm tilt servos reach fully-forward position within `90 / Q_TILT_RATE_DN` seconds.
- **Parent:** TILT-SyRS-0013.

---

**TILT-SyRS-0039**
Once tilting motors have reached the fully-forward position in fixed-wing mode, the tiltrotor system shall supply forward thrust to the tilting motors via `output_motor_mask()` using the throttle channel output and the `Q_TILT_MASK` as the active motor set.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `motors->output_motor_mask(current_throttle, mask, plane.rudder_dt)`.
- **Rationale:** Once fully forward, tilting motors function as fixed-wing thrust motors; routing throttle through the mask ensures only the tilting motors receive the FW thrust command.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — In fixed-wing mode at full forward tilt, apply throttle; confirm thrust is produced only on motors in `Q_TILT_MASK`.
- **Parent:** TILT-SyRS-0038.

---

**TILT-SyRS-0040**
When tilting motors have not yet reached the fully-forward position during the fixed-wing transition, the tiltrotor system shall apply the same slew-rate limit to throttle changes on those motors as it applies to the tilt angle itself, to prevent a step change in thrust.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `max_change = tilt_max_change(false); current_throttle = constrain_float(new_throttle, current_throttle - max_change, current_throttle + max_change)` in the partial-tilt branch.
- **Rationale:** Without throttle slewing, the tilting motors could jump from VTOL thrust to FW thrust in a single cycle as they pass the fully-forward threshold, causing a large pitch disturbance.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Monitor tilting motor throttle during FW transition; confirm throttle change per cycle does not exceed the tilt rate limit converted to throttle units.
- **Parent:** TILT-SyRS-0038.

---

**TILT-SyRS-0041**
When `Q_OPTIONS` bit 21 is set and the vehicle is disarmed and not in MANUAL mode, the tiltrotor system shall command tilt to 0° (fully upright hover position) to prevent rotor ground strikes.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `disarmed_tilt_up = !arming.is_armed_and_safety_off() && (control_mode != &mode_manual) && option_is_set(Option::DISARMED_TILT_UP)`.
- **Rationale:** When the vehicle is parked on the ground with tilting motors in the forward position, motor spin-up or wind gusts can cause propellers to contact the ground. Tilting up removes this hazard.
- **Stakeholder:** Safety Engineer; Ground Operations.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_OPTIONS` bit 21; disarm vehicle (not in MANUAL); confirm tilt servos move to 0°.
- **Parent:** TILT-SyRS-0038.

---

**TILT-SyRS-0042**
When `Q_TILT_FIX_ANGLE` is greater than 0° and `Q_TILT_FIX_GAIN` is greater than 0, the tiltrotor system shall apply differential tilt vectoring to the tilt servo outputs in fixed-wing flight to provide roll and pitch stabilisation authority, scaled by `Q_TILT_FIX_GAIN` and the throttle-to-airspeed ratio.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::vectoring()` — `no_yaw` branch; FW gain applied to elevon and elevator outputs.
- **Rationale:** In forward flight, tilt vectoring can supplement or replace aerodynamic control surfaces for roll and pitch, which is particularly useful at low airspeeds before control surfaces become effective.
- **Stakeholder:** Flight Control Engineer; Aerodynamics Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — In fixed-wing flight with `Q_TILT_FIX_ANGLE=10`, `Q_TILT_FIX_GAIN=0.5`, apply a roll input; confirm differential tilt output responds proportionally.
- **Parent:** TILT-SyRS-0015.

---

## 11. Transition: VTOL to Fixed-Wing (Forward Transition)

The transition state machine operates through three sequential states: `AIRSPEED_WAIT → TIMER → DONE`.

---

**TILT-SyRS-0043**
The tiltrotor system shall initiate a forward transition when VTOL assistance conditions are met (per TILT-SyRS-0067 through TILT-SyRS-0069) or when the pilot or autopilot commands a fixed-wing flight mode.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `AIRSPEED_WAIT` state entry condition.
- **Rationale:** The transition must begin in response to both autonomous (assistance) and commanded (mode change) triggers to ensure the vehicle reaches safe forward flight in all scenarios.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Switch from QHover to FBWA; confirm transition enters `AIRSPEED_WAIT` state immediately.
- **Parent:** TILT-SyRS-0012.

---

**TILT-SyRS-0044**
During the `AIRSPEED_WAIT` transition state, the tiltrotor system shall maintain VTOL motors at full spool (`THROTTLE_UNLIMITED`) until measured airspeed exceeds `AIRSPEED_MIN`.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `set_desired_spool_state(THROTTLE_UNLIMITED)` in `AIRSPEED_WAIT`; advance to `TIMER` when `aspeed > airspeed_min`.
- **Rationale:** VTOL motors provide lift and pitch authority during the acceleration phase; shutting down before minimum airspeed is reached risks an uncontrolled descent.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Monitor motor spool state during forward transition; confirm `THROTTLE_UNLIMITED` is maintained until airspeed reaches `AIRSPEED_MIN`.
- **Parent:** TILT-SyRS-0043.

---

**TILT-SyRS-0045**
When the tiltrotor has no dedicated fixed-wing forward motor (`_have_fw_motor = false`), the tiltrotor system shall inhibit TECS throttle integrator wind-up during the `AIRSPEED_WAIT` state.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `if (tiltrotor.enabled() && !tiltrotor.has_fw_motor()) plane.TECS_controller.reset_throttle_I()`.
- **Rationale:** Without a forward motor, TECS has no throttle authority during `AIRSPEED_WAIT`; integrator wind-up would cause a large throttle spike when TECS authority is restored after transition.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — In a no-FW-motor tiltrotor, monitor TECS throttle integrator during `AIRSPEED_WAIT`; confirm it remains at zero.
- **Parent:** TILT-SyRS-0043.

---

**TILT-SyRS-0046**
When the tilt type is not Vectored Yaw, the tiltrotor system shall use coordinated-turn yaw (body-frame yaw rate commanded to match the desired auto yaw rate) during the `AIRSPEED_WAIT` and `TIMER` states.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `if (!tiltrotor.is_vectored()) attitude_control->rate_bf_yaw_target(desired_auto_yaw_rate_cds(true))`.
- **Rationale:** Coordinated turns keep sideslip near zero during the acceleration phase, reducing drag and preventing lateral attitude divergence.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Perform a forward transition while turning; confirm sideslip angle remains within ±5° for non-vectored configurations.
- **Parent:** TILT-SyRS-0043.

---

**TILT-SyRS-0047**
When measured airspeed exceeds `AIRSPEED_MIN`, the tiltrotor system shall record the current tilt angle as `airspeed_reached_tilt` and shall advance the transition state to `TIMER`.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `airspeed_reached_tilt = tiltrotor.current_tilt; transition_state = State::TIMER`.
- **Rationale:** Recording the tilt angle at airspeed crossover enables smooth throttle blending in the TIMER phase between VTOL and FW thrust.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Log `airspeed_reached_tilt` during transition; confirm value is recorded at the instant airspeed first exceeds `AIRSPEED_MIN`.
- **Parent:** TILT-SyRS-0044.

---

**TILT-SyRS-0048**
During the `TIMER` transition state, the tiltrotor system shall reduce VTOL motor throttle linearly from `last_throttle` toward zero over `Q_TRANSITION_MS` milliseconds.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `transition_scale = (trans_time_ms - timer_ms) / trans_time_ms; throttle_scaled = last_throttle * transition_scale`.
- **Rationale:** Linear throttle reduction prevents a sudden unloading of VTOL motors that would cause a pitch disturbance and possible altitude loss.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Log VTOL motor throttle during TIMER phase; confirm it decreases monotonically to near zero over `Q_TRANSITION_MS` ms.
- **Parent:** TILT-SyRS-0047.

---

**TILT-SyRS-0049**
When a tiltrotor has no fixed VTOL motor and no fixed FW motor, the tiltrotor system shall blend VTOL throttle and fixed-wing throttle proportionally to tilt angle progress between `airspeed_reached_tilt` and the fully-forward position during the `TIMER` state.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `ratio = (current_tilt - airspeed_reached_tilt) / (get_fully_forward_tilt() - airspeed_reached_tilt); throttle_scaled = constrain_float(throttle_scaled*(1-ratio) + fw_throttle*ratio, 0, 1)`.
- **Rationale:** For all-tilting-motor configurations there is no separate FW motor; the transition from VTOL to FW thrust must be continuous to avoid a thrust gap or spike.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Monitor combined motor thrust during transition; confirm no step change exceeding 10% occurs at any point.
- **Parent:** TILT-SyRS-0048.

---

**TILT-SyRS-0050**
The tiltrotor system shall advance the transition state to `DONE` only when both the `Q_TRANSITION_MS` timer has expired and the tilt angle has reached its target as indicated by the `tilt_angle_achieved()` flag.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `if (transition_timer_ms > trans_time_ms && tilt_fwd_complete) transition_state = State::DONE`.
- **Rationale:** Requiring both conditions prevents premature DONE declaration when tilt lags behind the timer, which would transfer control to TECS before forward thrust is fully established.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Monitor transition logs; confirm `DONE` state is never reached before both `tilt_angle_achieved()=true` and the timer has expired.
- **Parent:** TILT-SyRS-0047.

---

**TILT-SyRS-0051**
At transition `DONE` state, the tiltrotor system shall set the TECS minimum throttle to the current forward motor throttle output and shall apply a throttle slew-rate limiter to prevent an instantaneous drop in thrust.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `plane.TECS_controller.set_throttle_min(throttle, true); SRV_Channels::set_slew_last_scaled_output(k_throttle, throttle*100)`.
- **Rationale:** Without this handoff, TECS starts from its default minimum throttle rather than the actual motor thrust level, causing a transient thrust reduction and pitch disturbance.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Monitor throttle channel output at the DONE transition; confirm no instantaneous change greater than 5% throttle occurs.
- **Parent:** TILT-SyRS-0050.

---

**TILT-SyRS-0052**
The tiltrotor system shall not apply the level-transition climb rate clamp (`Q_OPTIONS` bit 0) to tiltrotor configurations during the `AIRSPEED_WAIT` state.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `if (option_is_set(LEVEL_TRANSITION) && !tiltrotor.enabled()) climb_rate_cms = MIN(climb_rate_cms, 0.0f)`.
- **Rationale:** Clamping climb rate on a tiltrotor would prevent altitude maintenance during the acceleration phase, which can result in the vehicle descending into terrain.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_OPTIONS` bit 0; perform forward transition on a tiltrotor; confirm altitude hold is maintained during `AIRSPEED_WAIT`.
- **Parent:** TILT-SyRS-0043.

---

## 12. Transition: Fixed-Wing to VTOL (Back Transition)

---

**TILT-SyRS-0053**
When the vehicle enters any VTOL flight mode from a fixed-wing mode, the tiltrotor system shall reset the forward transition state machine to `AIRSPEED_WAIT` in preparation for the next forward transition.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::VTOL_update()` — `transition_state = State::AIRSPEED_WAIT`.
- **Rationale:** Resetting to `AIRSPEED_WAIT` ensures the full transition sequence executes on the next VTOL-to-FW switch, preventing skipping directly to `DONE` from a stale state.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Cycle VTOL → FW → VTOL → FW; confirm transition enters `AIRSPEED_WAIT` on the second FW switch.
- **Parent:** TILT-SyRS-0043.

---

**TILT-SyRS-0054**
When `Q_BACKTRANS_MS` is greater than 0, the tiltrotor system shall increase the pitch angle limit from 0° to the target maximum value linearly over `Q_BACKTRANS_MS` milliseconds upon entering a VTOL position-control mode from forward flight.

- **Source:** `ArduPlane/quadplane.h` — `back_trans_pitch_limit_ms`; `ArduPlane/quadplane.cpp` — back-transition pitch ramp logic.
- **Rationale:** An instantaneous full-pitch back-transition command causes a large deceleration spike that can overstress the airframe and cause passenger/payload disturbance.
- **Stakeholder:** Flight Control Engineer; Structural Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Perform a back transition with `Q_BACKTRANS_MS=3000`; confirm the pitch limit increases linearly over 3 seconds.
- **Parent:** TILT-SyRS-0043.

---

**TILT-SyRS-0055**
When `Q_BACKTRANS_MS` is set to 0, the tiltrotor system shall apply no pitch ramp and shall allow the full pitch limit immediately upon entering the VTOL mode.

- **Source:** `ArduPlane/quadplane.h` — `back_trans_pitch_limit_ms` default and zero-check.
- **Rationale:** Some airframes have structural margins that permit an immediate full-pitch back transition; the zero value disables the ramp for these cases.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_BACKTRANS_MS=0`; perform back transition; confirm pitch limit is at its maximum value from the first control cycle.
- **Parent:** TILT-SyRS-0054.

---

## 13. Transition Failure Handling

---

**TILT-SyRS-0056**
When `Q_TRANS_FAIL` is greater than 0 and the forward transition has remained in `AIRSPEED_WAIT` state for longer than `Q_TRANS_FAIL` seconds, the tiltrotor system shall transmit a CRITICAL severity GCS text message with the content "Transition failed, exceeded time limit".

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `gcs().send_text(MAV_SEVERITY_CRITICAL, "Transition failed, exceeded time limit")`.
- **Rationale:** The GCS alert provides the operator with timely notification of a transition failure, enabling manual intervention before the aircraft departs controlled flight.
- **Stakeholder:** Safety Engineer; Operator.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_TRANS_FAIL=5`; initiate a transition with insufficient airspeed; confirm CRITICAL GCS message is received within 5 seconds of transition start.
- **Parent:** TILT-SyRS-0043.

---

**TILT-SyRS-0057**
When `Q_TRANS_FAIL` expires on a tiltrotor and `Q_OPTIONS` bit 19 is set and ground speed exceeds 50% of `AIRSPEED_MIN`, the tiltrotor system shall force the transition to complete by advancing to the `TIMER` state rather than activating the failure action.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `tiltrotor_with_ground_speed = tiltrotor.enabled() && groundspeed() > airspeed_min * 0.5; if (option_is_set(TRANS_FAIL_TO_FW) && tiltrotor_with_ground_speed) transition_state = State::TIMER`.
- **Rationale:** Tiltrotors with adequate ground speed can complete a transition even without reaching the airspeed sensor threshold; forcing completion avoids an unnecessary emergency recovery on capable airframes.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Configure conditions to fail the airspeed check but with ground speed > 50% of `AIRSPEED_MIN`; confirm transition completes rather than triggering failure action.
- **Parent:** TILT-SyRS-0056.

---

**TILT-SyRS-0058**
When a transition failure is triggered and `Q_TRANS_FAIL_ACT` is set to 0 (QLAND), the tiltrotor system shall switch the vehicle to QLAND mode.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `case QLAND: plane.set_mode(mode_qland, VTOL_FAILED_TRANSITION)`.
- **Rationale:** QLAND is the safest immediate recovery action; it initiates a controlled vertical descent to land the vehicle at the current position.
- **Stakeholder:** Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_TRANS_FAIL_ACT=0`; trigger a transition timeout; confirm mode changes to QLAND within one control cycle.
- **Parent:** TILT-SyRS-0056.

---

**TILT-SyRS-0059**
When a transition failure is triggered and `Q_TRANS_FAIL_ACT` is set to 1 (QRTL), the tiltrotor system shall switch the vehicle to QRTL mode and shall set the position controller state to `QPOS_POSITION1`.

- **Source:** `ArduPlane/quadplane.cpp::SLT_Transition::update()` — `case QRTL: plane.set_mode(mode_qrtl, ...); poscontrol.set_state(QPOS_POSITION1)`.
- **Rationale:** QRTL commands the vehicle to return to launch; setting `QPOS_POSITION1` ensures the position controller starts from the correct initial deceleration state.
- **Stakeholder:** Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_TRANS_FAIL_ACT=1`; trigger a transition timeout; confirm mode changes to QRTL and position state is `QPOS_POSITION1`.
- **Parent:** TILT-SyRS-0056.

---

## 14. VTOL Assistance (Q_ASSIST)

---

**TILT-SyRS-0060**
When in a fixed-wing flight mode and airspeed falls below `Q_ASSIST_SPEED`, the tiltrotor system shall activate VTOL motor assistance.

- **Source:** `ArduPlane/VTOL_Assist.cpp::VTOL_Assist::should_assist()` — speed trigger.
- **Rationale:** Below the assistance speed, aerodynamic control surfaces lose authority; VTOL motors provide the supplementary lift and attitude control needed to prevent loss of control.
- **Stakeholder:** Safety Engineer; Flight Control Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Reduce airspeed below `Q_ASSIST_SPEED` in a fixed-wing mode; confirm VTOL motors spool up within 1 second.
- **Parent:** NONE.

---

**TILT-SyRS-0061**
When in a fixed-wing flight mode and attitude error in roll or pitch exceeds `Q_ASSIST_ANGLE` degrees for a continuous period of `Q_ASSIST_DELAY` seconds, the tiltrotor system shall activate VTOL motor assistance.

- **Source:** `ArduPlane/VTOL_Assist.cpp::VTOL_Assist::should_assist()` — angle error hysteresis trigger; `ArduPlane/quadplane.cpp` — `Q_ASSIST_ANGLE`, `Q_ASSIST_DELAY`.
- **Rationale:** An attitude error sustained beyond the threshold indicates the fixed-wing controllers have lost authority; VTOL motors can recover the vehicle before the attitude reaches a dangerous angle.
- **Stakeholder:** Safety Engineer; Flight Control Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Induce an attitude error exceeding `Q_ASSIST_ANGLE` for `Q_ASSIST_DELAY` seconds; confirm VTOL assistance activates.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0062**
When in a fixed-wing flight mode and altitude above ground level falls below `Q_ASSIST_ALT`, the tiltrotor system shall activate VTOL motor assistance.

- **Source:** `ArduPlane/VTOL_Assist.cpp::VTOL_Assist::should_assist()` — altitude error trigger; `ArduPlane/quadplane.cpp` var_info2 index 16.
- **Rationale:** Low-altitude flight near terrain requires the highest level of control authority; VTOL assistance ensures the vehicle can arrest descent before ground impact.
- **Stakeholder:** Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Descend below `Q_ASSIST_ALT` AGL in a fixed-wing mode; confirm VTOL assistance activates.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0063**
The tiltrotor system shall require each angle and altitude assistance trigger to remain active for `Q_ASSIST_DELAY` seconds before activating assistance, and shall maintain assistance until the trigger condition clears.

- **Source:** `ArduPlane/VTOL_Assist.h::VTOL_Assist::Assist_Hysteresis::update()` — hysteresis state machine.
- **Rationale:** Hysteresis prevents false positive assistance activations from momentary attitude transients, which would waste battery and produce unnecessary pilot alerts.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Apply a brief attitude error below `Q_ASSIST_DELAY` duration; confirm no assistance activation. Apply for full delay duration; confirm activation.
- **Parent:** TILT-SyRS-0061.

---

**TILT-SyRS-0064**
When `Q_ASSIST_SPEED` is set to -1, the tiltrotor system shall disable all Q_ASSIST triggers except during active transitions.

- **Source:** `ArduPlane/quadplane.cpp` — `Q_ASSIST_SPEED` parameter note; `VTOL_Assist::should_assist()` — speed = -1 check.
- **Rationale:** Some operators require full fixed-wing flight without any automatic VTOL assist; the -1 value provides an explicit disable while still permitting assistance during the mandatory transition phases.
- **Stakeholder:** Operator; Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_ASSIST_SPEED=-1`; reduce airspeed below normal assistance threshold; confirm VTOL motors remain at shutdown in fixed-wing flight.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0065**
When `Q_OPTIONS` bit 7 is set, the tiltrotor system shall force VTOL motor assistance to remain active at all times during fixed-wing flight.

- **Source:** `ArduPlane/quadplane.h::QuadPlane::Option` — bit 7 `Force_QASSIST`; `VTOL_Assist::should_assist()`.
- **Rationale:** Certain airframes or mission profiles require constant VTOL assistance for structural or controllability reasons; the force option provides this without requiring a sensor threshold.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_OPTIONS` bit 7; enter a fixed-wing mode; confirm VTOL motors remain active throughout.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0066**
When `Q_ASSIST_OPTIONS` bit 0 is set, the tiltrotor system shall disable the forced fixed-wing controller recovery action when Q_ASSIST is active.

- **Source:** `ArduPlane/VTOL_Assist.h::VTOL_Assist::OPTION::FW_FORCE_DISABLED` — bit 0.
- **Rationale:** Some operators prefer to retain full manual control over recovery rather than allowing the autopilot to force a fixed-wing recovery sequence.
- **Stakeholder:** Operator; Safety Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect `VTOL_Assist::check_VTOL_recovery()` for correct `FW_FORCE_DISABLED` guard.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0067**
When `Q_ASSIST_OPTIONS` bit 1 is set, the tiltrotor system shall disable the automatic spin recovery output.

- **Source:** `ArduPlane/VTOL_Assist.h::VTOL_Assist::OPTION::SPIN_DISABLED` — bit 1.
- **Rationale:** Spin recovery output may conflict with specific airframe recovery techniques; this option allows it to be disabled when an external recovery mechanism is provided.
- **Stakeholder:** Safety Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** I — Inspect `VTOL_Assist::output_spin_recovery()` for correct `SPIN_DISABLED` guard.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0068**
When `ModeQLoiter::run()` detects an extreme attitude upset via `check_VTOL_recovery()`, the tiltrotor system shall delegate the control loop for that cycle to `ModeQHover::run()`.

- **Source:** `ArduPlane/mode_qloiter.cpp::ModeQLoiter::run()` — `if (assist.check_VTOL_recovery()) { mode_qhover.run(); return; }`.
- **Rationale:** QHover uses simpler attitude stabilisation without position-hold error accumulation, which is better suited to recovering from extreme attitudes than QLoiter's position loop.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Induce a large attitude upset in QLoiter; confirm control transitions to QHover behaviour until attitude recovers.
- **Parent:** TILT-SyRS-0060.

---

## 15. VTOL Flight Modes

---

**TILT-SyRS-0069**
The tiltrotor system shall support the QStabilize flight mode, in which the pilot commands attitude directly via roll, pitch, and yaw sticks, and throttle commands motor power directly, and fixed-wing surfaces provide supplemental stabilisation.

- **Source:** `ArduPlane/mode_qstabilize.cpp::ModeQStabilize::run()`.
- **Rationale:** QStabilize is the fundamental VTOL control mode and the baseline from which all other VTOL modes are derived.
- **Stakeholder:** Flight Control Engineer; Operator.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** D — Enter QStabilize; demonstrate pitch, roll, and yaw response to stick inputs with motor thrust following throttle stick.
- **Parent:** NONE.

---

**TILT-SyRS-0070**
The tiltrotor system shall support the QHover flight mode, in which the position controller maintains altitude while the pilot commands horizontal attitude via roll and pitch sticks.

- **Source:** `ArduPlane/mode_qhover.cpp::ModeQHover::run()` — `hold_hover(get_pilot_desired_climb_rate_cms())`.
- **Rationale:** QHover reduces pilot workload by automating altitude hold while retaining manual horizontal control.
- **Stakeholder:** Flight Control Engineer; Operator.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Enter QHover; release throttle to mid; confirm altitude is maintained within ±1 m for 30 seconds.
- **Parent:** TILT-SyRS-0069.

---

**TILT-SyRS-0071**
The tiltrotor system shall support the QLoiter flight mode, in which the loiter controller maintains both position and altitude while the pilot can reposition the vehicle via roll and pitch sticks.

- **Source:** `ArduPlane/mode_qloiter.cpp::ModeQLoiter::run()` — loiter and position controller calls.
- **Rationale:** QLoiter enables hands-off hovering and controlled repositioning, which is essential for inspection and payload delivery missions.
- **Stakeholder:** Flight Control Engineer; Operator.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Enter QLoiter; release all sticks; confirm position maintained within ±2 m horizontal and ±1 m vertical for 60 seconds.
- **Parent:** TILT-SyRS-0069.

---

**TILT-SyRS-0072**
The tiltrotor system shall support the QAcro flight mode, in which the attitude control system commands body-frame angular rates proportional to stick input, with maximum rates bounded by `Q_ACRO_RLL_RATE`, `Q_ACRO_PIT_RATE`, and `Q_ACRO_YAW_RATE` in degrees per second.

- **Source:** `ArduPlane/quadplane.h` — `acro_roll_rate`, `acro_pitch_rate`, `acro_yaw_rate`; mode_qacro.cpp.
- **Rationale:** Rate control mode allows experienced pilots to perform manoeuvres beyond the angle limits of stabilised modes.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — In QAcro with `Q_ACRO_RLL_RATE=360`, apply full roll stick; confirm roll rate does not exceed 360 deg/s.
- **Parent:** TILT-SyRS-0069.

---

**TILT-SyRS-0073**
The tiltrotor system shall support the QAutotune flight mode, in which the attitude controller automatically adjusts PID gains, and shall command tilt to 0° throughout the autotune sequence as specified in TILT-SyRS-0032.

- **Source:** `ArduPlane/tiltrotor.cpp::continuous_update()` — autotune tilt lock; mode_qautotune.cpp.
- **Rationale:** Autotune requires a stable hover reference; any forward tilt would introduce coupled dynamics that corrupt the identification algorithm.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Enter QAutotune; confirm tilt remains at 0° and PID gains are updated at the end of the session.
- **Parent:** TILT-SyRS-0032.

---

**TILT-SyRS-0074**
The tiltrotor system shall support the QRTL flight mode, in which the vehicle climbs to `Q_RTL_ALT` metres above home altitude and returns to the home location using VTOL position control.

- **Source:** `ArduPlane/quadplane.cpp` — `qrtl_alt_m` parameter; mode_qrtl.cpp.
- **Rationale:** QRTL provides an autonomous recovery path that uses VTOL control for the entire return, avoiding the need for a fixed-wing landing.
- **Stakeholder:** Operator; Safety Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Activate QRTL; confirm vehicle climbs to `Q_RTL_ALT` and returns to home position using VTOL motors.
- **Parent:** TILT-SyRS-0069.

---

**TILT-SyRS-0075**
The tiltrotor system shall support the QLand flight mode, in which the vehicle descends vertically at a rate controlled by `Q_LAND_FINAL_SPD` metres per second below `Q_LAND_FINAL_ALT` metres, and shall declare landing complete when altitude change over a 4-second window is less than `Q_LAND_ALTCHG` metres.

- **Source:** `ArduPlane/mode_qloiter.cpp::ModeQLoiter::run()` — `landing_detect` block; `land_final_speed_ms`, `land_final_alt_m`, `landing_detect.detect_alt_change_m`.
- **Rationale:** A controlled descent rate and positive landing detection prevent hard landings and premature motor shutdown.
- **Stakeholder:** Flight Control Engineer; Safety Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Execute QLand from 20 m; confirm final descent rate matches `Q_LAND_FINAL_SPD` below `Q_LAND_FINAL_ALT`; confirm motor shutdown occurs only after landing detection.
- **Parent:** TILT-SyRS-0069.

---

## 16. Forward Throttle in VTOL Modes

---

**TILT-SyRS-0076**
The tiltrotor system shall use the parameter `Q_FWD_THR_USE` to control when forward throttle tilt commands are active: 0 = disabled in all VTOL modes, 1 = active in position-control modes only, 2 = active in all VTOL modes.

- **Source:** `ArduPlane/quadplane.h::QuadPlane::FwdThrUse` — enum OFF/POSCTRL/ALL; `tiltrotor.cpp::continuous_update()`.
- **Rationale:** Restricting forward throttle to position-control modes prevents unintentional forward acceleration in manual modes such as QStabilize.
- **Stakeholder:** Avionics Systems Engineer; Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set each value of `Q_FWD_THR_USE`; confirm tilt responds to forward demand only in the permitted modes.
- **Parent:** TILT-SyRS-0033.

---

**TILT-SyRS-0077**
The tiltrotor system shall use `Q_FWD_THR_GAIN` as the gain factor mapping forward acceleration demand in m/s² to forward throttle percentage, and shall apply this gain when computing the tilt demand per TILT-SyRS-0033.

- **Source:** `ArduPlane/quadplane.h` — `q_fwd_thr_gain`; `tiltrotor.cpp::continuous_update()` — `NEW` forward throttle method.
- **Rationale:** The gain provides a tunable relationship between the demanded forward acceleration and the actual forward tilt applied, allowing operators to adjust forward thrust aggressiveness.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Performance.
- **Revision:** 001.
- **Verification Method:** A — Verify that at a defined forward acceleration demand and `Q_FWD_THR_GAIN` value, the computed forward throttle percentage matches the expected product.
- **Parent:** TILT-SyRS-0033.

---

**TILT-SyRS-0078**
When a forward throttle RC channel is assigned (RC option 209), the tiltrotor system shall limit the manual forward throttle output to `Q_FWD_MANTHR_MAX` percent.

- **Source:** `ArduPlane/quadplane.h` — `fwd_thr_max`; `tiltrotor.cpp` — `forward_throttle_pct()`.
- **Rationale:** Without a maximum limit, a pilot could command full forward tilt in VTOL mode causing uncontrolled acceleration toward forward flight.
- **Stakeholder:** Safety Engineer; Operator.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Assign FWD_THR channel; apply full stick; confirm tilt output does not exceed the value corresponding to `Q_FWD_MANTHR_MAX` percent.
- **Parent:** TILT-SyRS-0035.

---

**TILT-SyRS-0079**
The tiltrotor system shall apply `Q_FWD_PITCH_LIM` degrees as the maximum forward pitch limit in VTOL mode to prevent the wing from generating negative lift.

- **Source:** `ArduPlane/quadplane.h` — `q_fwd_pitch_lim`; quadplane.cpp — pitch limit enforcement.
- **Rationale:** In VTOL flight, a forward pitch angle causes the wing to generate lift in the wrong direction, increasing the load on VTOL motors and risking altitude loss.
- **Stakeholder:** Flight Control Engineer; Aerodynamics Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — In VTOL mode with forward throttle active, confirm pitch angle does not exceed `Q_FWD_PITCH_LIM` degrees forward.
- **Parent:** TILT-SyRS-0076.

---

## 17. Tricopter-Specific Motor Requirements

---

**TILT-SyRS-0080**
The tricopter motor subsystem shall assign motor outputs as follows: front-right motor to MOT1, front-left motor to MOT2, and rear motor to MOT4.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::AP_MotorsTri::init()` — `add_motor_num(MOT1)`, `add_motor_num(MOT2)`, `add_motor_num(MOT4)`.
- **Rationale:** This is the standard tricopter motor numbering convention; deviating from it would produce incorrect thrust mixing and control axis assignment.
- **Stakeholder:** Avionics Systems Engineer; Mechanical Integration Engineer.
- **Type:** Design Constraint.
- **Revision:** 001.
- **Verification Method:** I — Inspect `init()` for correct motor number registration.
- **Parent:** TILT-SyRS-0001.

---

**TILT-SyRS-0081**
The tricopter motor subsystem shall compute thrust commands using the following mixing equations: `thrust_right = −0.5 × roll + 0.5 × pitch`, `thrust_left = +0.5 × roll + 0.5 × pitch`, `thrust_rear = −0.5 × pitch`.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::AP_MotorsTri::output_armed_stabilizing()` — thrust assignment lines.
- **Rationale:** These coefficients distribute roll and pitch demands across the three motors with equal authority and minimal cross-coupling.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify analytically that the mixing matrix satisfies zero net torque for pure throttle and correct sign/magnitude for pure roll, pitch, and combined inputs.
- **Parent:** TILT-SyRS-0080.

---

**TILT-SyRS-0082**
The tricopter motor subsystem shall compute the yaw pivot angle as `asin(yaw_thrust)` and shall constrain it to the range `−YAW_SV_ANGLE` to `+YAW_SV_ANGLE` degrees.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::output_armed_stabilizing()` — `_pivot_angle = safe_asin(yaw_thrust)` with limit flag.
- **Rationale:** The pivot angle is the exact rear servo position needed to produce the demanded yaw torque; the constraint prevents commanding beyond the servo's physical travel.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Apply maximum yaw demand; confirm `_pivot_angle` equals `YAW_SV_ANGLE` and the `limit.yaw` flag is set.
- **Parent:** TILT-SyRS-0081.

---

**TILT-SyRS-0083**
The tricopter motor subsystem shall divide the rear motor thrust command by `cos(pivot_angle)` to compensate for the reduction in vertical thrust component caused by the servo deflection.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::output_armed_stabilizing()` — `_thrust_rear = _thrust_rear / cosf(_pivot_angle)`.
- **Rationale:** When the rear motor is pivoted, its vertical thrust component is reduced by the cosine of the angle; dividing by this factor restores the commanded vertical thrust.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** A — Verify that for a pivot angle of 30°, `thrust_rear` is divided by `cos(30°) ≈ 0.866`, increasing the command by approximately 15.5%.
- **Parent:** TILT-SyRS-0082.

---

**TILT-SyRS-0084**
When the tricopter motor subsystem is used within ArduPlane, the arming check shall not require the tail servo (`k_motor7`) to be assigned.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::arming_checks()` — `#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)` guard suppressing servo assignment check.
- **Rationale:** ArduPlane performs its own servo assignment validation; the duplicated check in AP_MotorsTri would falsely fail arming when tiltrotor servo functions differ from standalone copter assignments.
- **Stakeholder:** Avionics Systems Engineer.
- **Type:** Design Constraint.
- **Revision:** 001.
- **Verification Method:** I — Inspect `arming_checks()` for the ArduPlane build guard.
- **Parent:** TILT-SyRS-0008.

---

**TILT-SyRS-0085**
The tricopter motor subsystem shall support a reverse tricopter configuration when `FRAME_TYPE` is set to `MOTOR_FRAME_TYPE_PLUSREV`, in which the single motor is at the front and the two motors are at the rear, and pitch mixing shall be inverted accordingly.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::init()` — `_pitch_reversed = frame_type == MOTOR_FRAME_TYPE_PLUSREV`.
- **Rationale:** Some tricopter designs place the single motor at the front; inverting pitch mixing ensures correct control direction for this layout.
- **Stakeholder:** Mechanical Integration Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set `FRAME_TYPE=PLUSREV`; apply a positive pitch demand; confirm the front motor responds as the single motor with inverted pitch sign.
- **Parent:** TILT-SyRS-0080.

---

**TILT-SyRS-0086**
The tricopter motor subsystem shall accept a differential rudder thrust input `rudder_dt` via `output_motor_mask()` and shall apply it to produce yaw in fixed-wing forward flight.

- **Source:** `libraries/AP_Motors/AP_MotorsTri.cpp::output_motor_mask()`.
- **Rationale:** In forward flight the yaw servo is replaced by differential thrust from the tilted motors; `rudder_dt` provides the rudder demand that drives this differential.
- **Stakeholder:** Flight Control Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — In FW mode with tricopter tiltrotor, apply rudder input; confirm differential thrust appears between left and right tilted motors.
- **Parent:** TILT-SyRS-0039.

---

## 18. Disarmed Behaviour

---

**TILT-SyRS-0087**
When the vehicle is disarmed, `Q_OPTIONS` bit 10 (DISARMED_TILT) is set, and the vehicle is in a VTOL mode, the tiltrotor system shall deflect the vectored-yaw tilt servos in response to rudder stick input to enable ground testing of servo direction.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::vectoring()` — disarmed vectored yaw block.
- **Rationale:** Servo direction verification before flight reduces the risk of yaw reversals during the first flight; disarmed testing with propellers removed is the only safe method.
- **Stakeholder:** Ground Operations; Integration Engineer.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Disarm; set bit 10; apply rudder input; confirm tilt servos deflect in the expected direction.
- **Parent:** TILT-SyRS-0007.

---

**TILT-SyRS-0088**
When `Q_OPTIONS` bit 10 is set, the tiltrotor system shall delay the activation of the disarmed vectored-yaw test by 3000 milliseconds after the last disarm event, to allow propellers to stop rotating before servo motion begins.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::vectoring()` — `constexpr uint32_t TILT_DELAY_MS = 3000; if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS)`.
- **Rationale:** Servo movement while propellers are still rotating could cause blade contact with servo linkages, creating a safety hazard.
- **Stakeholder:** Safety Engineer; Ground Operations.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Disarm; apply rudder input immediately; confirm no servo movement within the first 3000 ms; confirm movement begins after 3000 ms.
- **Parent:** TILT-SyRS-0087.

---

**TILT-SyRS-0089**
When the vehicle is disarmed, `Q_OPTIONS` bit 21 (DISARMED_TILT_UP) is set, and the active mode is not MANUAL, the tiltrotor system shall command all tilt servos to the VTOL-up position (0°) to prevent rotor ground strikes.

- **Source:** `ArduPlane/tiltrotor.cpp::Tiltrotor::continuous_update()` — `disarmed_tilt_up` flag and `slew(0.0)` branch.
- **Rationale:** Tilted-forward rotors extend beyond the airframe footprint on many designs; the VTOL position retracts them to within the safe parking envelope.
- **Stakeholder:** Safety Engineer; Ground Operations.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Disarm (not in MANUAL) with bit 21 set; confirm all tilt servo outputs reach 0° within `90 / Q_TILT_RATE_UP` seconds.
- **Parent:** TILT-SyRS-0041.

---

**TILT-SyRS-0090**
When `Q_OPTIONS` bit 11 (DELAY_SPOOLUP) is set, the tiltrotor system shall delay VTOL motor spoolup by 2000 milliseconds after arming.

- **Source:** `ArduPlane/quadplane.h::QuadPlane::Option` — bit 11 `DELAY_SPOOLUP`; quadplane.cpp spoolup logic.
- **Rationale:** A spoolup delay allows the pilot to confirm the vehicle is stable and positioned correctly before thrust is generated.
- **Stakeholder:** Operator; Ground Operations.
- **Type:** Functional.
- **Revision:** 001.
- **Verification Method:** T — Set bit 11; arm; confirm no motor thrust for 2000 ms; confirm normal spoolup begins at 2000 ms.
- **Parent:** NONE.

---

## 19. Arming Requirements

---

**TILT-SyRS-0091**
The tiltrotor system shall refuse to arm when `Q_ASSIST_SPEED` is set to 0, and shall generate a pre-arm failure message to alert the operator.

- **Source:** `ArduPlane/quadplane.cpp` — pre-arm check for `assist.speed == 0`.
- **Rationale:** A value of 0 is the factory default and indicates the parameter has not been configured; arming without assistance speed configured leaves the aircraft without a safety-critical protection threshold.
- **Stakeholder:** Safety Engineer; Operator.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set `Q_ASSIST_SPEED=0`; attempt to arm; confirm arming is rejected with a pre-arm message.
- **Parent:** TILT-SyRS-0060.

---

**TILT-SyRS-0092**
When `Q_OPTIONS` bit 18 (ARMVTOL) is set, the tiltrotor system shall refuse to arm unless the active flight mode is a VTOL mode or an AUTO mode with a VTOL takeoff as the first navigation command.

- **Source:** `ArduPlane/quadplane.h::QuadPlane::Option` — bit 18 `ARMVTOL`; ArduPlane arming checks.
- **Rationale:** Arming in a fixed-wing mode with VTOL motors could cause unexpected motor activation during a hand-launch; restricting arming to VTOL modes ensures a safe ground-to-flight sequence.
- **Stakeholder:** Safety Engineer; Operator.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Set bit 18; attempt to arm in FBWA; confirm rejection. Arm in QStabilize; confirm success.
- **Parent:** TILT-SyRS-0091.

---

**TILT-SyRS-0093**
When `Q_TKOFF_ARSP_LIM` is greater than 0 and airspeed exceeds `Q_TKOFF_ARSP_LIM` metres per second during VTOL takeoff, the tiltrotor system shall abort the takeoff and switch to QLAND mode.

- **Source:** `ArduPlane/quadplane.h` — `maximum_takeoff_airspeed_ms`; quadplane.cpp takeoff checks.
- **Rationale:** Excessive airspeed during VTOL takeoff indicates the vehicle is exposed to wind that exceeds its hover control margins; aborting prevents loss of control in the most vulnerable phase of flight.
- **Stakeholder:** Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Simulate airspeed exceeding `Q_TKOFF_ARSP_LIM` during takeoff; confirm automatic switch to QLAND.
- **Parent:** TILT-SyRS-0091.

---

**TILT-SyRS-0094**
When `Q_TKOFF_FAIL_SCL` is greater than 0 and the takeoff duration exceeds `expected_duration × Q_TKOFF_FAIL_SCL` seconds without achieving the target altitude, the tiltrotor system shall switch to QLAND mode.

- **Source:** `ArduPlane/quadplane.h` — `takeoff_failure_scalar`; quadplane.cpp takeoff failure check.
- **Rationale:** An abnormally long takeoff duration indicates a propulsion or control deficiency; switching to QLAND prevents continued low-altitude operation in a degraded state.
- **Stakeholder:** Safety Engineer.
- **Type:** Safety & Reliability.
- **Revision:** 001.
- **Verification Method:** T — Simulate a slow takeoff exceeding the scaled timeout; confirm switch to QLAND.
- **Parent:** TILT-SyRS-0091.

---
