// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.constants;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import java.util.List;

/**
 * CAD geometry for display only. Robot coordinates: +X forward, +Y left, +Z up, meters/radians.
 * The robot origin is the floor projection of the drive pose origin. Component poses refer to the
 * model origins after AdvantageScope's zeroed transforms, not the original CAD assembly origins.
 */
public final class MechanismVisualizationConstants {
  /** Each stage's robot-relative pose at HOME and cumulative travel / measured carriage travel. */
  public record ElevatorStage(Pose3d poseAtHome, double travelRatio) {}

  /** The component origin must lie on its shaft; the rotation axis is in its reference local frame. */
  public record RotaryComponent(
      Pose3d poseAtHomeAndReferenceAngle,
      Translation3d rotationAxis,
      double referenceAngleRadians,
      double angleSign) {}

  // TODO: Add one entry per separately exported moving elevator stage, in model component order.
  // Specify its origin/orientation at HOME (robot-relative, floor origin), and its cumulative travel
  // ratio relative to measured carriage height. Do not add the stages' ratios together.
  public static final List<ElevatorStage> ELEVATOR_STAGES = List.of();

  // TODO: Set the pivot shaft pose at elevator HOME and a known encoder angle, the local rotation
  // axis, that reference angle in radians, and encoder-to-model rotation sign (+1 or -1).
  public static final RotaryComponent PIVOT =
      new RotaryComponent(null, null, Double.NaN, Double.NaN);

  // TODO: Set the gripper model origin's rigid translation (m) and rotation (rad) in the pivot frame.
  // Include arm length here; jaw opening animation is not represented by this transform.
  public static final Transform3d PIVOT_TO_GRIPPER = null;

  // TODO: Set the indexer shaft pose in the robot frame, its local rotation axis, a known encoder
  // reference angle in radians, and encoder-to-model rotation sign (+1 or -1).
  public static final RotaryComponent SPIN_INDEXER =
      new RotaryComponent(null, null, Double.NaN, Double.NaN);

  /** Unfilled geometry suppresses component display only, never mechanism control. */
  public static boolean isConfigured() {
    return !ELEVATOR_STAGES.isEmpty()
        && ELEVATOR_STAGES.stream().allMatch(stage -> stage.poseAtHome() != null
            && Double.isFinite(stage.travelRatio()) && stage.travelRatio() > 0)
        && rotaryConfigured(PIVOT) && rotaryConfigured(SPIN_INDEXER)
        && PIVOT_TO_GRIPPER != null;
  }

  private static boolean rotaryConfigured(RotaryComponent component) {
    return component.poseAtHomeAndReferenceAngle() != null
        && component.rotationAxis() != null
        && Double.isFinite(component.rotationAxis().getNorm())
        && component.rotationAxis().getNorm() > 0
        && Double.isFinite(component.referenceAngleRadians())
        && Math.abs(component.angleSign()) == 1;
  }

  private MechanismVisualizationConstants() {}
}
