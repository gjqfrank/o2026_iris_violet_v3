// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.sim;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import frc.robot.constants.MechanismVisualizationConstants;
import frc.robot.constants.MechanismVisualizationConstants.RotaryComponent;
import frc.robot.subsystems.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/** Measured component poses, independent of control and alliance. No field transform is applied. */
public final class MechanismVisualization {
  private static final Pose3d[] NO_COMPONENTS = new Pose3d[0];

  /** Array order: configured elevator stages, pivot/arm, gripper body, spin indexer. */
  public static void log(Distance elevatorHeight, Angle pivotAngle, Angle indexerAngle) {
    boolean configured = MechanismVisualizationConstants.isConfigured();
    Logger.recordOutput("Visualization/MechanismConfigured", configured);
    double travel = elevatorHeight.in(Meters) - ElevatorConstants.HOME_POSITION.in(Meters);
    double pivotRadians = pivotAngle.in(Radians);
    double indexerRadians = indexerAngle.in(Radians);
    if (!configured || !Double.isFinite(travel)
        || !Double.isFinite(pivotRadians) || !Double.isFinite(indexerRadians)) {
      Logger.recordOutput("Visualization/MechanismPoses", NO_COMPONENTS);
      return;
    }

    var stages = MechanismVisualizationConstants.ELEVATOR_STAGES;
    Pose3d[] poses = new Pose3d[stages.size() + 3];
    for (int i = 0; i < stages.size(); i++) {
      var stage = stages.get(i);
      poses[i] = raisedPose(stage.poseAtHome(), travel * stage.travelRatio());
    }
    Pose3d pivot = rotatedPose(MechanismVisualizationConstants.PIVOT, pivotRadians, travel);
    poses[stages.size()] = pivot;
    poses[stages.size() + 1] = pivot.transformBy(MechanismVisualizationConstants.PIVOT_TO_GRIPPER);
    poses[stages.size() + 2] =
        rotatedPose(MechanismVisualizationConstants.SPIN_INDEXER, indexerRadians, 0);
    Logger.recordOutput("Visualization/MechanismPoses", poses);
  }

  private static Pose3d raisedPose(Pose3d poseAtHome, double travelMeters) {
    // Elevator travel is along robot +Z, regardless of the exported model's local orientation.
    return new Pose3d(poseAtHome.getTranslation().plus(new Translation3d(0, 0, travelMeters)),
        poseAtHome.getRotation());
  }

  private static Pose3d rotatedPose(RotaryComponent component, double angleRadians, double travel) {
    Translation3d axis = component.rotationAxis();
    Rotation3d rotation = new Rotation3d(VecBuilder.fill(axis.getX(), axis.getY(), axis.getZ()),
        component.angleSign() * (angleRadians - component.referenceAngleRadians()));
    // Apply shaft rotation in the component frame; continuous indexer angles naturally wrap here.
    return raisedPose(component.poseAtHomeAndReferenceAngle(), travel)
        .transformBy(new Transform3d(new Translation3d(), rotation));
  }

  private MechanismVisualization() {}
}
