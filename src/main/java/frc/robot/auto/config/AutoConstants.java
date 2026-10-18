// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.auto.config;

// import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;


/** BACK SIDE auto uses blue-origin field coordinates and the existing Teleop arm poses. */
public final class AutoConstants {
  public static final int BLUE_BACK_SIDE_TAG_ID = 21;
  public static final int RED_BACK_SIDE_TAG_ID = 10;
  public static final String APPROACH_PATH_NAME = "Sushi Pirate Auto"; // TODO: Validate this imported approach path starts at the physical parking position, clears SCORING SIDES and ends within local alignment range of BACK SIDE.
  public static final boolean RESET_POSE_TO_PATH_START = true; // TODO: Confirm physical starting position/heading matches the path's ideal starting pose; true retains the imported auto's resetOdom behavior.

  // Auto pacing: per-action bounds and chassis-stillness checks, used by the Superstructure's
  // auto entry points and by every auto sequence step.
  public static final double MOTION_TIMEOUT_SECONDS = Double.NaN; // TODO: Maximum wait per action for autonomous sequencing, s; NaN disables the auto-only bound while teleop holds at a step instead.
  public static final double DRIVE_STABLE_SECONDS = Double.NaN; // TODO: Continuous time below both drive stationary speed limits before an auto reef action may start, s.
  public static final double DRIVE_MAX_LINEAR_SPEED = Double.NaN; // TODO: Maximum measured translation speed magnitude considered stationary, m/s.
  public static final double DRIVE_MAX_ANGULAR_SPEED = Double.NaN; // TODO: Maximum absolute angular speed considered stationary; check both chassis-derived and gyro-measured speed, rad/s.

  public static int backSideTagId() {
    return AllianceFlipUtil.shouldFlip() ? RED_BACK_SIDE_TAG_ID : BLUE_BACK_SIDE_TAG_ID;
  }

  /** Match AutoBuilder's red conversion, including a path's explicit preventFlipping flag. */
//   public static Optional<Pose2d> approachStartingPose(PathPlannerPath path) {
//     return path.getStartingHolonomicPose()
//         .map(pose -> path.preventFlipping ? pose : AllianceFlipUtil.apply(pose));
//   }

  private AutoConstants() {}
}
