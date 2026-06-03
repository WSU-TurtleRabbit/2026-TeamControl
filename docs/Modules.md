WorldStateEstimator / DynamicObstacleMap
- consumes raw snapshots
- estimates velocities
- builds obstacle map with clearance
- provides path-clear checking and planner graph
- does not decide robot commands

PlannerManager
- owns per-robot PlannerState
- checks direct path first
- uses direct target if path is clear
- reroutes only if target moved, path blocked, or waypoint blocked
- outputs active_target_pose

PDController
- owns per-robot PDState
- receives current_pose, active_target_pose, pd_mode
- outputs vx, vy, w, reached_target, errors
- handles previous error and acceleration/deceleration limits

ActuatorController
- uses actuator_mode, ball distance, angle, and PD output
- outputs dribble/kick
- stateless for now

SkillIntentExecutor
- loops through active robot SkillIntents
- resolves pd_mode and actuator_mode
- asks PlannerManager for active target
- calls PDController
- calls ActuatorController
- calls CommandComposer
- sends RobotCommand + runtime to Dispatcher
