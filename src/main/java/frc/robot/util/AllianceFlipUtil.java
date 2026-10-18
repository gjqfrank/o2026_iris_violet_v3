// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.util;

import static org.wpilib.units.Units.*;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

import frc.robot.constants.FieldConstants;

public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength.in(Meters) - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth.in(Meters) - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.PI) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static boolean shouldFlip() {
    return MatchState.getAlliance().isPresent()
        && MatchState.getAlliance().get() == Alliance.RED;
  }
}
