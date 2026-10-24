# Limelight 3A auto-align

This branch adds new vision files and a separate **Limelight Align Test** OpMode.
It does not edit MainTeleOp, intake/shooter code, motor diagnostics, build files,
or wheel settings. It was started from `master` in its own checkout so the team
can keep working on the wheel branch.

## What is implemented

- Choose red with D-pad left or blue with D-pad right before Start. The selection
  locks at Start. An unselected alliance prevents automatic steering.
- Hold R1 for automatic chassis rotation toward the chosen cell opening. Release
  R1 for manual steering; deliberate right-stick turning overrides alignment.
- Ignore opposing-alliance, unknown, uncalibrated, upside-down, very small,
  behind-camera, or out-of-range tag observations.
- Group calibrated tags by cell. Keep the selected cell while it is eligible,
  including when different tags in the same cell become visible. A different cell
  requires three fresh frames before steering resumes.
- Transform each tag's measured opening offset using its 3D pose, the camera
  mounting transform, and the shooter exit position. Do not aim at a random tag
  center and do not use the pipeline's global tx to steer toward a different ID.
- Check pipeline number, total age including latency, and changing camera
  timestamps. Losing a valid target immediately removes automatic turn demand.
- Require three fresh acquisition frames, cap turn demand, and suspend steering
  after a large discontinuity while the new observations are acquired.
- Indicate **ALIGNED (HEADING ONLY)** after fresh images remain within the angular
  tolerance for the settling interval. This is not an automatic-fire signal.

The test mode provides robot-centric translation and manual rotation at a 25%
wheel-power limit after its drivetrain settings have been verified. It does not
operate the shooter, intake, or stopper and does not change field-centric driving.
It does not drive to an out-of-view goal or infer alliance from robot location.

## Starting configuration

The initial configuration intentionally runs the test mode as **vision only**:

- `CAMERA_MOUNT_VERIFIED = false`
- Camera and shooter measurements are `Double.NaN`.
- There are no tag-to-opening calibration entries yet.
- `TEST_DRIVE_VERIFIED = false`

The team has confirmed the shooter fires forward. Photos help check mounting,
but final geometry needs measurements. Calibrated tags can be added one at a
time; an uncalibrated ID is never used to steer.

All settings for this feature are in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/autoalign/AutoAlignConfig.java`.

## 1. Configure the Limelight

1. Configure the connected device as a **Limelight 3A** in the active FTC robot
   configuration and name it `limelight` (or change `LIMELIGHT_NAME`).
2. In Limelight's web interface, configure pipeline **0** as a fiducial/AprilTag
   pipeline using **36h11** and a tag side of **82.55 mm (3.25 inches)**. Use the
   individual printed tag size, not the entire four-tag sticker size.
3. Allow IDs 30 through 45; include all four tags in each cell. Do not set a
   priority ID or sorting/filter configuration that hides other cell detections.
   Ensure per-fiducial JSON results and 3D target-camera poses are being reported.
4. Use a camera calibration appropriate for the installed lens/resolution.
   Tune exposure/gain on the actual field so tags remain detectable in motion.
5. Upload/run **Limelight Align Test**. Select an alliance during Init. Verify
   the visible IDs and result age in telemetry, with test driving still disabled.

The hardware adapter calls `setPollRateHz(100)`, `pipelineSwitch(0)`, and `start()`.
Its matching `stop()` is called when the test mode stops.

| Cell | Alliance | Individual IDs |
| --- | --- | --- |
| Scoring-table side | Red | 30, 31, 32, 33 |
| Audience side | Red | 34, 35, 36, 37 |
| Audience side | Blue | 38, 39, 40, 41 |
| Scoring-table side | Blue | 42, 43, 44, 45 |

BIOBUZZ's cells tip. These tags are moving target references: do not upload them
as static field landmarks and use their apparent movement as robot localization.
This feature uses target-relative poses, not botpose or a static field map.

## 2. Measure the geometry

Use meters and degrees. Choose one fixed robot origin (for example the chassis
center projected onto the floor) for both positions:

- `CAMERA_POSITION`: camera lens coordinates in **(right, forward, up)**.
- `SHOOTER_EXIT_POSITION`: the ball exit center in the same axes and origin.
- `CAMERA_ROLL_CLOCKWISE_DEG`: clockwise looking through the camera.
- `CAMERA_PITCH_UP_DEG`: positive when the optical axis points upward.
- `CAMERA_YAW_RIGHT_DEG`: positive when the camera points right of robot forward.
- `SHOOTER_YAW_RIGHT_DEG`: 0 for the confirmed forward-facing shooter.

Angles are applied roll, pitch, then yaw. Use a rigid camera mount. Validate the
transform against known points on the real field before marking the mount verified.
These local axes are stated explicitly because SDK classes wrap Limelight-native
pose numbers; they must not be assumed to use FTC VisionPortal's axis convention.

For EACH tag ID that should be usable, measure or obtain from CAD:

- The vector from the tag center to the desired opening aim point, in that tag's
  native Limelight **(right, down, out-of-plane)** coordinates.
- The cluster's printed-up unit vector in that same tag frame. `(0, -1, 0)`
  applies only if that individual tag's printed up agrees with cluster up.

Add an entry inside `settings()` using measured numbers:

```java
new TagCalibration(tagId,
        new Vector(measuredRight, measuredDown, measuredOut),
        new Vector(clusterUpX, clusterUpY, clusterUpZ))
```

The example names above are placeholders, not dimensions. Do not enter zero
opening offsets merely to make the calibration warning go away. Offsets must be
defined for individual tags, so changing which tag is visible does not move the
aim point to a different part of the sticker.

The controller rotates the cluster-up vector into robot space and requires its
dot product with robot-up to exceed 0.15. This is the equivalent geometric idea
behind checking upright cluster roll, **not** blindly applying VisionPortal's
`ftcPose.roll` threshold to Limelight's different Euler fields. Test both tipped
states and each configured tag to verify the convention and reject the downward cell.

## 3. Verify steering after the wheel work

1. Finish the wheel branch's motor-name and direction checks first.
2. Copy the verified four motor names/directions into this feature's
   `TEST_MOTOR_NAMES` and `TEST_MOTOR_REVERSED` settings, then set
   `TEST_DRIVE_VERIFIED = true`. This only enables the separate test OpMode.
3. With the robot supported and wheels clear, verify manual turn direction and
   verify that a target on the shooter's right produces a turn toward the right.
   Check `turnSign` if your verified drivetrain convention differs. Do not invert
   existing drive directions to compensate for an aiming-sign error.
4. With the robot on the floor and area clear, select your alliance during Init,
   press Start, then release R1 and center the sticks once to enable controls.
5. Hold R1 while an eligible calibrated cell is visible. Translation remains
   driver-controlled. Right-stick turning beyond 0.15 overrides automatic aiming.
6. Confirm releasing R1, covering the camera, disconnecting vision, viewing only
   opposing tags, and changing pipeline all remove automatic steering. A driver
   turn override remains available even with no vision.
7. Confirm the other cell is rejected when upside down, and target loss or a cell
   switch clears the aligned indication and requires new acquisition frames.
8. Tune `kp`, `maxTurn`, `toleranceDeg`, and `settleMs` from observed behavior.
   Default values are starting points, not robot-tuned gains.

`ALIGNED` refers to measured horizontal heading only. Shooter speed, hood angle,
ball flight, motion compensation, and automatic feed control are separate work.
Vision pose estimates alone do not establish that a shot will score.

## MainTeleOp integration after the wheel merge

An alignment button in the existing match-driving OpMode requires editing
**MainTeleOp.java**. That edit has not been made because the team asked to avoid
conflicts with the wheel branch. The reusable controller already returns a turn
demand and can be integrated without replacing the wheel branch's mixing math.

The eventual integration points are:

1. Construct/start `LimelightAutoAlign` and `AutoAlignController` during Init.
2. Add D-pad alliance selection in `init_loop()` and lock it at Start.
3. After reading the driver's turn stick, call `controller.update(...)` with the
   new frame, selected alliance, R1, raw manual turn, translation demand, and
   `System.nanoTime()`.
4. While R1 is held and the driver is not overriding, replace only the turn
   input with the controller's automatic turn, or zero if it is blocked.
5. Add the alignment/vision telemetry and stop camera polling in `stop()`.

Wait until the team finishes/merges that file before making these changes.
Neither the controller nor the adapter writes to motors or starts a mechanism.

## Validation

Run `tests/autoalign/run.ps1` to compile/run the pure-Java controller checks.
They cover alliance and orientation filtering, wrong pipeline, stale/frozen
frames, release/override/target loss, settling only on fresh frames, cell locking
and switches, turn caps/signs, and camera/shooter/tag geometry.

Build the Android app with `gradlew.bat :TeamCode:assembleDebug`. The tests and
build do not replace camera calibration and drivetrain tests on the robot.

## References

- [Limelight 3A quick start](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [Limelight coordinate systems](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems)
- [Limelight point-of-interest tracking](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d)
- [FIRST BIOBUZZ cluster targeting guidance](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html)
- [BIOBUZZ field manual, section 9.9](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20V1.htm)
