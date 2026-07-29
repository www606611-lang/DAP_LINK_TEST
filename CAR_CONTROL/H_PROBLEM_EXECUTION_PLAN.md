# H Problem Execution Plan

Status: active source-of-truth plan from 2026-07-29.

Official source:

```text
D:/qq/Download/2026江苏赛区大学生电子设计竞赛赛题pdf/
2026江苏赛区大学生电子设计竞赛赛题pdf/
车载平衡滚球运动控制系统（H题）.pdf
```

If this plan conflicts with an older generic competition plan, the official H
problem and this document win. Existing validated modules remain reusable
baselines, but their former demonstrations are not evidence that an H item has
passed.

## 1. Exact Scored Requirements

| ID | Required behavior | Official limit | Internal target |
| --- | --- | --- | --- |
| H1 | Live transmit a clear view of the entire groove, record every run, and replay it | Stable real-time display and complete recording | Independent control timing; receiver records every scored run |
| H2 | Start at A by button, follow clockwise for one lap, stop at A, stop and display timer | `<=20 s`, stop error `<=2 cm` | `<=18 s`, stop error `<=1.2 cm` |
| H3 | While chassis is stationary, move ball `O -> +5 cm -> -5 cm` and stabilize at `-5 cm` | `<=5 s`, maximum absolute error at both targets `<=1 cm` | `<=4.2 s`, maximum absolute error `<=0.6 cm` |
| H4 | Start at A with ball at O, travel clockwise through B, hold ball near O | A-to-B `<=8 s`, maximum ball error `<=1 cm` | `<=7 s`, maximum ball error `<=0.6 cm` |
| H5 | Start at A with ball at O, travel one clockwise lap through A, hold ball near O | `<=30 s`, maximum ball error `<=1 cm` | `<=27 s`, maximum ball error `<=0.6 cm` |
| H6 | Start at A with ball at an arbitrary specified position, travel one clockwise lap through A, hold that position | `<=30 s`, maximum ball error `<=1 cm` | `<=27 s`, maximum ball error `<=0.6 cm` |

The internal targets provide measurement and judge-observation margin. Passing
an internal target does not replace a complete physical run under official
geometry.

## 2. Non-Negotiable Physical Rules

- Track centerline consists of two `1.5 m` straights and two `0.5 m` radius
  semicircles. Total centerline length is approximately `6.142 m`.
- Track line width is `1.8 +/- 0.2 cm`.
- A has a centered, perpendicular start/stop bar: `5 cm` long and the same line
  width. The judging reference is a `0.1 cm` wide, `30 cm` long centerline.
- Vehicle dimensions must not exceed `35 cm x 25 cm`, must use wheel drive, and
  must use an onboard battery.
- One unique judging mark must be placed on the vehicle center axis. Start and
  stop error are measured from this mark.
- No human intervention or remote driving is allowed after the start button is
  pressed. The vehicle projection must not completely leave the track line.
- Line sensing may use only infrared photoelectric modules. Quantity is not
  limited.
- A start button and an onboard display no larger than 2 inches are mandatory.
  Button press starts the timer; elapsed time must be visible.
- The beam is a straight `25 cm` length of 4-fen PPR pipe, approximately `2 cm`
  outside diameter and `0.34 cm` wall thickness. Its inside surface remains
  smooth and unmodified. No added friction material or pits are allowed.
- The steel ball is approximately `1 cm` in diameter. The scale is placed on
  the groove edge, not inside it, at `0.1 cm` spacing.
- Ball position measurement must use a camera.
- The camera view must cover the complete beam and clearly show the ball path.
  The video receiver/display/storage equipment remains outside the route and is
  submitted with the vehicle.
- The beam pivot is at least `5 cm` above the chassis plate. The complete beam
  must remain within the vehicle envelope.

The H2 time limit implies an average centerline speed above `0.307 m/s`. H4
requires more than `0.188 m/s` average over AB. Encoder PPS values are not
converted to competition speed until wheel circumference and loaded-ground
slip are measured.

## 3. Selected System Ownership

### Tianmengxing chassis

Tianmengxing remains the sole owner of both wheel motors and every chassis
inner loop. H-specific application code will own:

- task selection, start-button arbitration, timer, and result state;
- H-route line following and A start/finish-line recognition;
- lap arming, A/B passage timestamps, and the H2 precision stop;
- the under-2-inch competition display;
- high-level mission synchronization with K230;
- immediate wheel stop on button, local fault, or K230 balance fault.

The accepted speed, encoder, position, Heading, IMU, and line-sensor modules are
reused. H behavior is added in an `app` mission layer. The accepted five-corner
route behavior is not retuned in place.

### K230 balance subsystem

K230 owns the camera observation, ball-state estimation, beam controller,
MCP2515 transport, and the beam actuator. The existing two-axis target tracker
is retained as infrastructure/history, but H mode is a separate one-dimensional
ball-on-beam application.

Only the actuator axis mechanically connected to the right end of the beam is
enabled in H mode. The unused gimbal axis stays disabled unless the final
mechanism gives it a concrete H-task role.

### ESP32-C3 link

The accepted link carries only bounded high-level synchronization and status:

- chassis to K230: `PROFILE`, `BALL_TARGET_0P1MM`, `ARM`, `START`, `STOP`,
  mission sequence, and optional timestamped chassis acceleration feedforward;
- K230 to chassis: `READY`, `ARMED`, `RUNNING`, signed `BALL_X_0P1MM`,
  `BALL_V_0P1MM_S`, target error, observation age/quality, beam angle, current
  control state, maximum run error, and `FINISHED/FAULT`;
- both directions: heartbeat, sequence, CRC, and link age.

K230 does not send wheel PWM, wheel speed, or direct line-control commands.
Tianmengxing does not send CAN frames or stepper pulses. The PC is never a
motion controller during a judged run.

### Video receiver

Video display/recording is a monitoring path, not a control-loop dependency.
The control observation must continue at its required rate if the receiver is
slow or disconnected. K230 records a local copy where practical; the receiver
also records the complete run for H1 evidence.

## 4. Vision Redesign For H

The current steel-ball model is only a pipeline proof:

- training images show the ball on varied desk, paper, hand, book, and floor
  backgrounds, not in the official PPR groove;
- the live confidence reported by the operator is commonly around `0.5`, close
  to the current `0.45` acquisition threshold;
- the current full two-dimensional pipeline averages about `21.5 FPS`, with
  normal vision work around `32-35 ms` and periodic frames around `114-122 ms`;
- the current target-loss policy stops motion after `120 ms`, which would turn
  a short detection gap into a beam-control discontinuity.

That baseline is not accepted for H3-H6.

The first H-oriented software pass on 2026-07-29 changed single-class
postprocessing to a highest-score fast path, limited target coasting to two
frames, throttled only the overlay, removed the fixed 20-frame GC, and added
stage/percentile telemetry. A motor-disabled K230 run improved from about
`21.73 FPS` to `35.99 FPS`; ordinary vision work was `26-30 ms`, with p95/p99
around `25-30 ms`. The remaining automatic MicroPython collection occurred
after roughly 30 seconds and produced a measured `100 ms` maximum. This is
useful progress but still fails the final no-stall gate; the final pipe ROI must
reduce allocation further through a rectangular model or geometric fast path.

### H observation pipeline

1. Mount the camera rigidly on the chassis above the beam so the complete 25 cm
   groove remains in view over the full actuator-angle range. The camera does
   not move with the beam.
2. Locate the beam ends/edges in the image and combine them with actuator-angle
   feedback. Because the chassis camera is fixed while the beam tilts, the
   control ROI and pixel-to-centimeter transform follow the projected beam
   axis instead of assuming one static horizontal scale. Calibrate the left
   end, O, right end, and visible edge scale.
3. Use a narrow control ROI around the groove instead of a square full-scene
   inference image. Initial benchmark sizes are `320x96` and `320x128`.
4. Benchmark a fast geometric detector on the fixed groove: background/edge
   contrast, connected components, circularity, expected diameter, and the
   predicted X window.
5. Train a pipe-specific rectangular detector as reacquisition or fallback.
   Training data must contain the final PPR beam, final camera mount, all target
   positions, beam angles, lighting, reflections, chassis vibration, motion
   blur, and negative frames without a ball.
6. Fuse accepted measurements with an alpha-beta or Kalman state estimator for
   `x` and `v`. One or two missed frames use bounded prediction; a single miss
   never forces an abrupt zero command.
7. A longer invalid interval commands a neutral bounded beam state and asks the
   chassis mission to stop. It must never reuse an indefinitely stale ball
   position.

The final detector is selected by measured latency and position error, not by
model novelty. Camera-based classical measurement satisfies the official
camera rule and is preferred as the fast path when it passes the data gate.

### Vision acceptance gate

- Fresh ball-position output rate: at least `50 Hz`.
- Camera-to-position latency: p95 `<=20 ms`, p99 `<=30 ms`.
- End-to-actuator-command latency: p95 `<=35 ms`.
- No periodic control stall above `50 ms` in a 60-second run.
- Static position RMS error `<=0.15 cm`; maximum error `<=0.4 cm` at all marked
  positions and permitted beam angles.
- Dynamic missed-frame rate below `0.5%`; no unhandled loss burst longer than
  two frames in the official lighting/motion dataset.
- If a learned fallback is used, its acquisition confidence distribution must
  be measured on held-out pipe runs. Lowering the threshold alone is not an
  accepted fix for low confidence.

## 5. Ball-On-Beam Control

The H controller is a cascade, not the existing image-centering velocity map:

```text
camera x -> calibrated position/velocity estimator
         -> ball position controller
         -> requested beam angle/angle rate
         -> bounded actuator trajectory
         -> ZDT internal position loop and feedback
```

- Coordinates are signed centimeters from O; positive direction follows the
  official beam scale.
- The outer controller begins with state feedback or PD on ball position and
  velocity. Integral action is added only for measured steady bias and remains
  clamped.
- Beam angle, speed, acceleration, and mechanical end positions are bounded.
- Setpoint changes use a trajectory generator so H3 reaches `+5 cm`, confirms
  its error bound, then reverses to `-5 cm` without an uncontrolled impulse.
- Center hold and arbitrary-position hold use the same reusable API.
- Chassis acceleration/yaw telemetry may be added as feedforward after the
  camera-only loop is stable. Wireless telemetry is never required for local
  K230 stability.
- Controller logs retain timestamp, measured X, estimated velocity, target,
  beam command, actuator feedback, observation quality, and maximum error.

## 6. H-Route Chassis Mission

The former five-corner route and its wide-line corner recovery do not match the
official stadium-shaped track. H mode gets a separate mission policy while
calling the accepted line and wheel APIs.

The chassis code audit on 2026-07-29 confirmed that the line loop alone is not
a complete H solution. Production layering is:

```text
H mission: profile, A/B events, timer, lap validation, precision stop, K230 interlock
    -> line tracking: line error to left/right speed targets
        -> wheel speed: encoder PI to motor output
```

Wheel position, odometry, Heading, and Yaw controllers remain reusable for
precision stopping, validation, and later feedforward; deleting them would save
little linked Flash and remove needed H infrastructure. Temporary speed,
position, Yaw, Heading, and timed line bring-up workflows are now excluded from
normal GCC/TIClang builds through `CAR_ENABLE_BRINGUP=OFF`. PB4/PB5 no longer
start old Yaw demonstrations from idle. The existing PB21 line mission remains
only a supervised chassis test and must be replaced, not renamed, by the H
mission state machine below.

Required route logic:

1. At A, detect the initial perpendicular bar but do not count it as a lap.
2. Start the visible timer on the debounced PB21 press.
3. Arm finish detection only after leaving the A bar and satisfying minimum
   odometry/distance/time conditions.
4. Run clockwise on two straights and two smooth `0.5 m` radius semicircles.
5. Record B passage from calibrated odometry plus curve-entry evidence.
6. On the next valid A bar, latch lap time. H2 transitions to a bounded
   position-stop profile referenced to the vehicle judging mark; H5/H6 record
   passage and then stop safely after the scored event.
7. Reject isolated wide sensor patterns that are inconsistent with lap
   odometry, so reflections or line overlap cannot end the run early.

The line controller must be tuned on a full-size official track. A successful
run on the older short five-corner tape route is not an H-route acceptance.

## 7. Competition Interaction

Before motion:

- PB4/PB5 select the H profile and adjust the H6 ball target through a small
  menu; PB21 confirms/starts.
- The display shows profile, ball target, K230 readiness, line readiness,
  battery/link health, and `READY`.

During motion:

- PB21 or either expansion key is an immediate stop request.
- The display prioritizes elapsed time, profile, lap phase, ball X/target/error,
  observation age/quality, beam/controller state, line state, and fault state.
- Tuner commands may observe telemetry but may not change motion or controller
  parameters during a judged run. The final competition build disables host
  motion commands.

After completion:

- The display freezes total or AB time, result, stop error estimate, and maximum
  ball error until the operator resets the profile.
- H1 video and the synchronized CSV log are retained under one run identifier.

## 8. Mission Profiles

| Profile | Chassis behavior | Ball behavior | Completion |
| --- | --- | --- | --- |
| H2 | One clockwise lap and precision stop at A | Balance system may stay neutral | A bar plus bounded stop |
| H3 | Wheels remain disabled/high-Z | `O -> +5 cm -> -5 cm`, settle | Both error windows and total time pass |
| H4 | A through B clockwise | Hold O | B passage recorded; stop safely afterward |
| H5 | One clockwise lap through A | Hold O | A passage recorded; stop safely afterward |
| H6 | One clockwise lap through A | Hold selected signed position | A passage recorded; stop safely afterward |

Every profile uses one mission sequence and explicit `READY -> ARMED -> RUNNING
-> FINISHED/FAULT`. Wheels start only after K230 acknowledges the required ball
profile as armed. A K230 fault or stale link stops the chassis; a chassis stop
commands the beam to its bounded neutral/hold state.

## 9. Implementation And Physical Acceptance Gates

| Gate | Work | Physical acceptance evidence |
| --- | --- | --- |
| H0 | Measure vehicle envelope; build official PPR beam, pivot, actuator linkage, scale, camera mount, and judging mark | Fits `35x25 cm`, `h>=5 cm`, beam remains inside vehicle, ball rolls freely |
| H1V | Capture final-pipe dataset; calibrate ROI/centimeters; benchmark fast and learned detectors | Vision gate in section 4 passes under static, moving, glare, shadow, and vibration cases |
| H1T | Implement live receiver plus complete recording without blocking control | 60-second live display and replay with synchronized run ID; control timing unchanged |
| H2A | Calibrate actuator position to beam angle; enforce mechanical bounds | Repeated bounded angle moves, stop/fault tests, no impact or lost motor state |
| H3 | Tune stationary ball cascade and H3 trajectory | Ten consecutive `O -> +5 -> -5 cm` runs meet time/error limits |
| H2C | Build full-size stadium route, A marker detector, lap timer, and precision stop | Ten laps; all `<=20 s`, stop error `<=2 cm`, no false finish |
| H4 | Coordinate A-to-B chassis run and center hold | Ten runs meet `8 s / 1 cm` limits |
| H5 | Full-lap center hold | Ten runs meet `30 s / 1 cm` limits |
| H6 | Full-lap arbitrary target hold across representative positions | At least three signed targets, ten runs each, meet `30 s / 1 cm` limits |
| HR | Endurance, reset/fault matrix, video/log archive, and report tables | Repeated power-on and judged-sequence rehearsal with complete evidence |

Firmware is committed and pushed only after the user accepts the corresponding
physical gate. Experimental profiles remain isolated until promoted.

## 10. Report Evidence

The 20 report points require work throughout implementation, not a final-day
summary. Retain:

- line-control and ball-on-beam model derivation;
- circuit/communication block diagrams and program state diagrams;
- detector calibration and latency distributions;
- ball step response, maximum error, settling time, and loss statistics;
- lap/AB times, stop-error measurements, and pass-rate tables;
- test setup, lighting, battery voltage, payload, and failure analysis;
- synchronized video and CSV run identifiers.

## 11. Explicit Non-Goals

- The electromagnet is not part of H1-H6 and remains disabled unless a later
  documented H mechanism gives it a necessary role.
- Two-axis image-centering and unlimited gimbal rotation are not the H balance
  controller.
- Generic K230 wireless ownership of chassis motion is omitted. The H mission
  coordinator stays on Tianmengxing; K230 exchanges only balance synchronization
  and status.
- The existing `1400 pps` five-corner acceptance is preserved but is not copied
  into H mode as an assumed final speed.
- A lower neural-network confidence threshold is not treated as a latency or
  reliability solution.

## 12. Immediate Next Work

The next blocking artifact is the final PPR beam/camera/actuator geometry. The
current desk-background steel-ball dataset can seed experiments, but final
training and latency tuning begin only after the camera sees the official pipe
over its full travel. In parallel, the chassis can add a disabled H mission
skeleton and host tests, but ground tuning must use the official full-size
stadium route.
