# Beaking time

TODO:::::::::

- [x] work
- [x] support FoC
- [x] create motor control requests
- [ ] motor config requests/classes
    - [x] ClosedLoop: wrap
    - [x] Current Limits
    - [x] Fused CANCoder of some sort
    - [x] Limit switch configs
        - [x] Support external (DIO) pins
    - [x] Motion Magic
    - [x] Duty Cycle: deadband, peak forward/reverse, open/closed loop ramps
    - [x] Same for voltage
    - [ ] Same for torque
    - [ ] Soft Limits
    - [ ] Similar thing for encoders/gyros
    - [ ] Nominal voltage
    - [ ] PID
        - Include slots in the config
    - [ ] Conversion API
    - [ ] Brake/Coast?
- [x] swerve control requests
    - [ ] Facing Angle
    - [ ] X-Drive
    - [ ] Robot Centric
    - [ ] SysId control of some sort
- [ ] look forward to prevent slips and such
- [ ] Collision detection; ignore odometry
- [ ] Slip detection
- [ ] Real feedforward
- [ ] Standardized Simulation
- [ ] Differential drivetrain requests
- [x] TorqueCurrent output
- [ ] More clean API for duty vs voltage vs torque
- [ ] General cleanup of old stuff
- [ ] 250Hz odometry
    - depends on status signals
- [ ] PWM controller w/ DIO encoder?
- [ ] Fused CANCoder

- [x] General DataSignal improvements
    - [x] getter for value/timestamp
    - [x] Update frequencies
    - [x] Refresh
    - [ ] RefreshAll
    - [x] SignalStore, perhaps?

- [ ] LogStore & DashboardStore
- [ ] BeakSparkFLEX
  - Wait for motor controller API to be done.

- [x] Get rid of BeakGyroSubsystem
- [ ] VisionSystem API
- [ ] Acceleration control