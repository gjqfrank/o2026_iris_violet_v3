// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.constants;

import org.wpilib.math.geometry.Translation2d;

/** Calibration for local alignment, always in the blue-origin 2025 Welded field frame. */
public final class ReefAlignConstants {
  public enum RobotSide { LEFT, RIGHT }

  /** LEFT/RIGHT are viewed by a driver facing the selected Reef face. */
  public enum ReefBranch { LEFT, RIGHT }

  // TODO: All four offsets locate the target robot center in the selected AprilTag 2D frame, not camera extrinsics.
  // Tag +X points out of Reef; -Y toward the left branch, +Y toward the right branch. Units: m; include jaw placement relative to robot center.
  // RobotSide identifies the robot side facing Reef; ReefBranch left/right is viewed by a driver facing that Reef face.
  public static final Translation2d leftSideLeftBranchOffset = null; // TODO: Target robot-center (X,Y) offset from the selected face AprilTag for robot LEFT side / LEFT branch; tag frame, m. Include the jaw offset from robot center.
  public static final Translation2d leftSideRightBranchOffset = null; // TODO: Target robot-center (X,Y) offset from the selected face AprilTag for robot LEFT side / RIGHT branch; tag frame, m. Include the jaw offset from robot center.
  public static final Translation2d rightSideLeftBranchOffset = null; // TODO: Target robot-center (X,Y) offset from the selected face AprilTag for robot RIGHT side / LEFT branch; tag frame, m. Include the jaw offset from robot center.
  public static final Translation2d rightSideRightBranchOffset = null; // TODO: Target robot-center (X,Y) offset from the selected face AprilTag for robot RIGHT side / RIGHT branch; tag frame, m. Include the jaw offset from robot center.

  // Conservative starting limits, to be validated on the robot.
  public static final double maxLinearVelocityMetersPerSec = 0.5; // TODO: Alignment translation speed limit, m/s; constrains the remaining-distance profile and the magnitude of the final translation velocity vector.
  public static final double maxLinearAccelerationMetersPerSecSq = 0.5; // TODO: Alignment translation acceleration limit, m/s^2; constrains the remaining-distance profile and the final velocity-vector change per cycle.
  public static final double maxAngularVelocityRadPerSec = 0.5; // TODO: Alignment angular speed limit, rad/s; constrains the heading profile and absolute final angular velocity output.
  public static final double maxAngularAccelerationRadPerSecSq = 1.0; // TODO: Alignment angular acceleration limit, rad/s^2; constrains the heading profile and final angular velocity change per cycle.
  public static final double velocityFeedforwardZeroDistanceMeters = 0.10; // TODO: Target distance at or below which translation and heading profile-velocity feedforward are zero, m; PID correction remains active. Tune final approach on the robot.
  public static final double velocityFeedforwardFullDistanceMeters = 0.15; // TODO: Target distance at or above which profile-velocity feedforward reaches full strength, m; scale linearly between the zero/full distances. Must exceed the zero distance; tune on the robot.
  public static final double maxEntryLinearVelocityMetersPerSec = 0.15; // TODO: Maximum measured translation speed magnitude when bumper alignment starts, m/s; faster entry is rejected. Must not exceed the alignment speed limit.
  public static final double maxEntryAngularVelocityRadPerSec = 0.2; // TODO: Maximum measured absolute angular speed when bumper alignment starts, rad/s; faster entry is rejected. Must not exceed the alignment angular speed limit.
  public static final double maxVisionAgeSeconds = 0.25; // TODO: Maximum age of the latest accepted vision pose measurement for bumper alignment and LT/RT, s; stale vision pauses alignment until a fresh measurement arrives.
  public static final double positionToleranceMeters = 0.02; // TODO: Shared distance tolerance for stopping translation correction and alignment arrival, m; final output still decelerates under the acceleration limit, and arrival also requires heading and measured speed tolerances.
  public static final double headingToleranceRadians = Math.toRadians(1.0); // TODO: Shared heading tolerance for stopping rotation correction and alignment arrival; enter degrees here and convert to rad.
  public static final double linearVelocityToleranceMetersPerSec = 0.05; // TODO: Maximum measured translation speed magnitude for alignment arrival, m/s; prevents declaring arrival while passing through the target.
  public static final double angularVelocityToleranceRadPerSec = 0.05; // TODO: Maximum measured absolute angular speed for alignment arrival, rad/s; prevents declaring arrival while still rotating.
  public static final double settleTimeSeconds = 0.15; // TODO: Continuous time satisfying position, heading, translation-speed and angular-speed arrival limits before reporting alignment success, s.
  public static final double controlPeriodSeconds = 0.02;

  public static Translation2d targetOffset(RobotSide side, ReefBranch branch) {
    if (side == RobotSide.LEFT) {
      return branch == ReefBranch.LEFT ? leftSideLeftBranchOffset : leftSideRightBranchOffset;
    }
    return branch == ReefBranch.LEFT ? rightSideLeftBranchOffset : rightSideRightBranchOffset;
  }

  private ReefAlignConstants() {}
}
