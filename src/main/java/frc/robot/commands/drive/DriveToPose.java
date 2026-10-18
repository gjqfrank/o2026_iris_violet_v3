// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.commands.drive;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at the
// root directory of this project.

import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import frc.robot.constants.Constants;
import frc.robot.constants.ReefAlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveToPose implements Command {
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius");

  static {
    drivekP.initDefault(4.0); // 4.0
    drivekD.initDefault(0.0);
    thetakP.initDefault(6.5); // 4.5
    thetakD.initDefault(0.0);
    driveMaxVelocity.initDefault(ReefAlignConstants.maxLinearVelocityMetersPerSec);
    driveMaxAcceleration.initDefault(ReefAlignConstants.maxLinearAccelerationMetersPerSecSq);
    thetaMaxVelocity.initDefault(ReefAlignConstants.maxAngularVelocityRadPerSec);
    thetaMaxAcceleration.initDefault(ReefAlignConstants.maxAngularAccelerationRadPerSecSq);
    driveTolerance.initDefault(ReefAlignConstants.positionToleranceMeters);
    thetaTolerance.initDefault(ReefAlignConstants.headingToleranceRadians);
    ffMinRadius.initDefault(ReefAlignConstants.velocityFeedforwardZeroDistanceMeters);
    ffMaxRadius.initDefault(ReefAlignConstants.velocityFeedforwardFullDistanceMeters);
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  @AutoLogOutput(key = "DriveToPose/AtGoal")
  private boolean atGoalFlag = false;

  private static BooleanSupplier manualModeSupplier = () -> false;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;

    private static Supplier<Translation2d> linearFF = () -> Translation2d.ZERO;
  private static DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void run(Coroutine coroutine) {
        initialize();
        while (true) {
            execute();
            coroutine.yield();
        }
    }

    @Override
    public void onCancel() {
        end(true);
    }

    @Override
    public String name() {
        return "DriveToPose";
    }

    @Override
    public Set<Mechanism> requirements() {
        return Set.of(drive);
    }

  public static void setManualSupplier(
      Supplier<Translation2d> linear, DoubleSupplier omega, BooleanSupplier goalSupplier) {
    linearFF = linear;
    omegaFF = omega;
    manualModeSupplier = goalSupplier;
  }

  public void initialize() {
    Pose2d currentPose = drive.getPose();
    ChassisVelocities fieldVelocity =
        drive.getChassisVelocities().toFieldRelative(drive.getRotation());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vx, fieldVelocity.vy);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .orElse(Rotation2d.ZERO)
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omega);
    lastSetpointTranslation = currentPose.getTranslation();

    atGoalFlag = false;
  }

  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current pose and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    // Calculate drive speed

    // currentPose relative to targetPose
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // use a small feedforward scaler if distance to target is small （0.10-0.15m？)
    double ffScaler =
        Math.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;

    // reset integral term which we don't need, the setpoint should not change because we pass in
    // the same value
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);

    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(
                driveErrorAbs,
                0.0); // this updates the current setpoint = the current distance to target

    // don't move if at goal
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    // goal is always 0,
    // setpoint is current distance to goal
    // lastSetpointTranslation = targetpose + difference from current pose to target
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose
                    .getTranslation()
                    .minus(targetPose.getTranslation())
                    .getAngle()
                    .orElse(Rotation2d.ZERO)) // angle from target to current pose
            .transformBy(Util.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // let's say current pose is (1,1), target pose is (0,0)
    // currentPose - targetPose = (1,1), its rotation is 45 degrees CCW of x-axis
    // we want to drive in the (-1,-1) direction,
    // so we multiply by driveVelocityScalar, which is always a NEGATIVE number
    // because our PID goal is 0 and setpoint is positive (distance to goal is always positive)
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose
                    .getTranslation()
                    .minus(targetPose.getTranslation())
                    .getAngle()
                    .orElse(Rotation2d.ZERO))
            .transformBy(Util.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Command speeds
    boolean isManualMode = manualModeSupplier.getAsBoolean();
    if (isManualMode) { // manual override
      Translation2d manualDriveVelocity = linearFF.get();
      // we dont finetune heading because:
      // 1. we have hardcoded headings this year, not moving targets
      // 2. we have no heading correction from vision, only pigeon
      // therefore we can always trust pigeon to align with hardcoded targets, no more omegaFF
      double manualThetaVelocity = thetaVelocity;
      Translation2d robotRelative = manualDriveVelocity.rotateBy(drive.getRotation().unaryMinus());
      drive.runVelocity(
          new ChassisVelocities(robotRelative.getX(), robotRelative.getY(), manualThetaVelocity));
    } else {
      Translation2d robotRelative = driveVelocity.rotateBy(drive.getRotation().unaryMinus());
      drive.runVelocity(
          new ChassisVelocities(robotRelative.getX(), robotRelative.getY(), thetaVelocity));
    }

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(thetaController.getSetpoint().position)));
    Logger.recordOutput("DriveToPose/Goal", targetPose);
  }

  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }


  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** True while the command lifecycle is inside execute; replaces seafoam's Lombok getter. */
  public boolean isRunning() {
    return running;
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
