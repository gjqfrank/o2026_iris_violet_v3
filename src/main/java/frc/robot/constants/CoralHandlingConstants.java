// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import frc.robot.util.ReefTargetColors.Level;
import frc.robot.constants.ReefAlignConstants.RobotSide;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ArmPose;

/** All heights use the elevator's homed reference; pivot angles use its finite absolute reference. */
public final class CoralHandlingConstants {
  private CoralHandlingConstants() {}

  public static final boolean AUTOMATION_CONFIGURED = false; // TODO: Master switch for automated Coral actions; enable after validating all enabled poses, timing and collision clearance.
  public static final Distance INDEXER_MIN_SAFE_HEIGHT = Meters.of(Double.NaN); // TODO: Minimum elevator height at which the turntable can rotate and pivot can sweep its full allowed range without hitting the turntable; meters relative to the homed height reference.
  public static final Distance INDEXER_HEIGHT_MARGIN = Meters.of(Double.NaN); // TODO: Positive clearance allowance for measurement error and mechanism flex, m; rotation requires MIN_SAFE_HEIGHT plus this margin.
  public static final ArmPose STOW = new ArmPose(
      Meters.of(Double.NaN), // TODO: Shared empty/loaded stow height relative to the homed reference, m; at least minimum safe height + clearance margin + elevator position tolerance to retain clearance during turntable rotation.
      Degrees.of(Double.NaN)); // TODO: Vertically-down arm target, deg in the calibrated pivot encoder reference; do not assume 0 degrees. Also used for turntable storage and auto Reef transit.
  public static final ArmPose INDEXER_PICKUP = new ArmPose(
      Meters.of(Double.NaN), // TODO: Elevator contact height for POV pickup from the aligned turntable slot, m relative to the homed reference.
      Degrees.of(Double.NaN)); // TODO: Pivot contact angle for POV slot pickup; rotate to this angle at safe height before descending and clamping, deg in the calibrated pivot encoder reference.
  public static final ArmPose INDEXER_STORE = new ArmPose(
      Meters.of(Double.NaN), // TODO: Elevator release height for A storage into the aligned empty slot, m relative to the homed reference; open the jaw only after descending to this height.
      STOW.pivot()); // Storage shares the vertically-down STOW.pivot() angle; no separate angle calibration.
  public static final Angle VERTICAL_UP_PIVOT_ANGLE = Degrees.of(Double.NaN); // TODO: Vertically-up pivot target for RT retreat and uncertain-gripper recovery, deg in the calibrated pivot reference. RT lowers and rotates here concurrently after the jaw opens; recovery raises to clearance first. Validate the swept space and sensor isolation from nearby Coral.
  public static final Distance GRAB_ENTRY_HEIGHT_DROP = Meters.of(Double.NaN); // TODO: Positive vertical offset below each Reef level's shared grab/place height, m. Empty B/X/Y and auto preparation subtract this distance; LT raises by it at the grab angle before clamping.
  public static final Distance PLACE_EXIT_HEIGHT_DROP = Meters.of(Double.NaN); // TODO: Positive elevator descent after RT opens the jaw, m; descend while rotating to VERTICAL_UP_PIVOT_ANGLE, then confirm EMPTY. The resulting height must retain turntable clearance plus height arrival tolerance throughout the sweep.

  // LEFT/RIGHT means the ROBOT side facing Reef, not the selected Reef branch.
  // These four angles are shared by L2/L3/L4. Empty jaws approach at PLACE, then extract at PREPARATION.
  public static final Angle LEFT_REEF_PREPARATION_ANGLE = Degrees.of(Double.NaN); // TODO: Loaded preparation angle when the robot LEFT side faces Reef; also the final LT extraction/confirmation angle after clamping. Shared by L2/L3/L4; deg in the calibrated pivot reference. Verify the Coral clears the rod and remains readable here.
  public static final Angle LEFT_REEF_PLACE_ANGLE = Degrees.of(Double.NaN); // TODO: Placement angle when the robot LEFT side faces Reef; also the empty-jaw B/X/Y approach angle for pickup. RT opens at this angle; shared by L2/L3/L4, deg in the calibrated pivot reference.
  public static final Angle RIGHT_REEF_PREPARATION_ANGLE = Degrees.of(Double.NaN); // TODO: Loaded preparation angle when the robot RIGHT side faces Reef; also the final LT extraction/confirmation angle after clamping. Shared by L2/L3/L4; deg in the calibrated pivot reference. Verify the Coral clears the rod and remains readable here.
  public static final Angle RIGHT_REEF_PLACE_ANGLE = Degrees.of(Double.NaN); // TODO: Placement angle when the robot RIGHT side faces Reef; also the empty-jaw B/X/Y approach angle for pickup. RT opens at this angle; shared by L2/L3/L4, deg in the calibrated pivot reference.

  public static final Distance L2_HEIGHT = Meters.of(Double.NaN); // TODO: Shared L2 grab/place height, m relative to home, for both robot sides. LT clamps and RT places here; empty preparation subtracts GRAB_ENTRY_HEIGHT_DROP, and RT retreat subtracts PLACE_EXIT_HEIGHT_DROP.
  public static final Distance L3_HEIGHT = Meters.of(Double.NaN); // TODO: Shared L3 grab/place height, m relative to home, for both robot sides. LT clamps and RT places here; empty preparation subtracts GRAB_ENTRY_HEIGHT_DROP, and RT retreat subtracts PLACE_EXIT_HEIGHT_DROP.
  public static final Distance L4_HEIGHT = Meters.of(Double.NaN); // TODO: Shared L4 grab/place height, m relative to home, for both robot sides. LT clamps and RT places here; empty preparation subtracts GRAB_ENTRY_HEIGHT_DROP, and RT retreat subtracts PLACE_EXIT_HEIGHT_DROP.

  public static final double OPEN_WAIT_SECONDS = Double.NaN; // TODO: Wait after commanding the jaw open before retreating, s; measure the slower opening time under actual operating air pressure.
  public static final double CLAMP_WAIT_SECONDS = Double.NaN; // TODO: Wait after commanding the jaw closed before extracting Coral, s; measure actual cylinder closing time.
  public static final double MOTION_TIMEOUT_SECONDS = Double.NaN; // TODO: Maximum wait for each elevator/pivot motion or slot alignment stage, s; jaw stages add their opening/closing wait. Exceeding the limit aborts the sequence.
  public static final double SENSOR_TIMEOUT_SECONDS = Double.NaN; // TODO: Maximum wait for a fresh color/EMPTY confirmation after leaving a slot or completing Reef retreat, s; discard old detection first and do not commit successful inventory on timeout.
  public static final double INDEXER_SETTLE_SECONDS = Double.NaN; // TODO: Continuous time with slot angle error and turntable speed within SpinIndexer transfer limits before the arm may descend into the slot, s.
  public static final double DRIVE_STABLE_SECONDS = Double.NaN; // TODO: Continuous time below both drive stationary speed limits before LT/RT may start, s; separate from alignment arrival settling time.
  public static final double DRIVE_MAX_LINEAR_SPEED = Double.NaN; // TODO: Maximum measured translation speed magnitude considered stationary for LT/RT; checked before and throughout the action, m/s.
  public static final double DRIVE_MAX_ANGULAR_SPEED = Double.NaN; // TODO: Maximum absolute angular speed considered stationary for LT/RT; check both chassis-derived and gyro-measured speed, rad/s.
  public static final double DRIVE_FEEDBACK_MAX_AGE_SECONDS = Double.NaN; // TODO: Maximum age of module and gyro motion feedback used to confirm stationary drive for LT/RT, s; older data cannot confirm stationary status.
  public static final double REEF_POSITION_TOLERANCE_METERS = Double.NaN; // TODO: Maximum planar distance between robot center and the last successfully aligned bumper target during LT/RT, m; the same target is checked after bumper release.
  public static final double REEF_HEADING_TOLERANCE_RADIANS = Double.NaN; // TODO: Maximum absolute heading error from the last successfully aligned bumper target during LT/RT, rad.
  public static final double TRIGGER_THRESHOLD = 0.5; // TODO: LT/RT input threshold for triggering one pickup/place request; unitless in 0..1. Release does not cancel; validate driver preference.

  public static Distance reefHeight(Level level) {
    return switch (level) { case L2 -> L2_HEIGHT; case L3 -> L3_HEIGHT; case L4 -> L4_HEIGHT; };
  }

  public static Angle reefPreparationAngle(RobotSide side) {
    return side == RobotSide.LEFT ? LEFT_REEF_PREPARATION_ANGLE : RIGHT_REEF_PREPARATION_ANGLE;
  }

  public static Angle reefPlaceAngle(RobotSide side) {
    return side == RobotSide.LEFT ? LEFT_REEF_PLACE_ANGLE : RIGHT_REEF_PLACE_ANGLE;
  }

  public static boolean positive(double value) {
    return Double.isFinite(value) && value > 0;
  }

  public static boolean clearanceConfigured() {
    return STOW.configured() && Double.isFinite(INDEXER_MIN_SAFE_HEIGHT.in(Meters))
        && INDEXER_MIN_SAFE_HEIGHT.in(Meters) >= ElevatorConstants.MIN_POSITION.in(Meters)
        && positive(INDEXER_HEIGHT_MARGIN.in(Meters))
        && STOW.height().in(Meters)
            >= INDEXER_MIN_SAFE_HEIGHT.in(Meters) + INDEXER_HEIGHT_MARGIN.in(Meters)
                + ElevatorConstants.POSITION_TOLERANCE.in(Meters);
  }
}
