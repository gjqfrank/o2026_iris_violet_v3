// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.commands.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.command3.Command;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.system.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTuning;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private DriveCommands() {}

  static Translation2d calculateLinearVelocityFromJoysticks(
      double x, double y, double maxLinearSpeedMetersPerSec) {
    // A square joystick gate can report a magnitude above one near the corners. Clamp before
    // shaping so the requested translation magnitude cannot exceed its selected speed limit.
    double linearMagnitude =
        Math.min(MathUtil.applyDeadband(Math.hypot(x, y), DriveTuning.deadband()), 1.0);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    // Square magnitude for finer control at low joystick deflections.
    return new Translation2d(linearMagnitude * linearMagnitude, linearDirection)
        .times(maxLinearSpeedMetersPerSec);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier maxLinearSpeedSupplier) {
    return joystickDriveWithAngularVelocity(
        drive,
        xSupplier,
        ySupplier,
        maxLinearSpeedSupplier,
        () -> {
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveTuning.deadband());
          return Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();
        });
  }

  // Angular velocity is in rad/s; translation uses the driver's field reference frame.
  private static Command joystickDriveWithAngularVelocity(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier maxLinearSpeedSupplier,
      DoubleSupplier angularVelocitySupplier) {
    return drive.runRepeatedly(
        () -> {
          double maxLinearSpeed =
            Math.clamp(
              maxLinearSpeedSupplier.getAsDouble(),
              0.0,
              drive.getMaxLinearSpeedMetersPerSec());
          Translation2d linearVelocity =
            calculateLinearVelocityFromJoysticks(
              xSupplier.getAsDouble(), ySupplier.getAsDouble(), maxLinearSpeed);
          ChassisVelocities speeds =
            new ChassisVelocities(
              linearVelocity.getX(),
              linearVelocity.getY(),
              angularVelocitySupplier.getAsDouble());
          Logger.recordOutput("Drive/Teleop/LinearSpeedLimit", maxLinearSpeed, MetersPerSecond);
          Rotation2d fieldReference =
            AllianceFlipUtil.shouldFlip()
              ? drive.getRotation().plus(new Rotation2d(Math.PI))
              : drive.getRotation();
          Translation2d robotRelative =
            new Translation2d(speeds.vx, speeds.vy).rotateBy(fieldReference.unaryMinus());
          drive.runVelocity(
            new ChassisVelocities(robotRelative.getX(), robotRelative.getY(), speeds.omega));
        })
      .named("Drive:Joystick");
  }

  /** Runs the forward current ramp; fit the logged torque current, position and velocity in SysId. */
  public static Command feedforwardCharacterization(Drive drive) {
    return drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD);
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(
        DriveConstants.WHEEL_RADIUS_RAMP_RATE.in(RadiansPerSecondPerSecond));
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return drive.run(
            coroutine -> {
              limiter.reset(0.0);
              double startTime = Timer.getTimestamp();
              boolean measuring = false;
              while (true) {
                double speed =
                    limiter.calculate(DriveConstants.WHEEL_RADIUS_MAX_VELOCITY.in(RadiansPerSecond));
                drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                if (!measuring && Timer.getTimestamp() - startTime >= 1.0) {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                  measuring = true;
                } else if (measuring) {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                }
                coroutine.yield();
              }
            })
        .whenExited(
            () -> {
              drive.stop();
              if (state.positions == null) return;
              double[] positions = drive.getWheelRadiusCharacterizationPositions();
              double wheelDelta = 0.0;
              for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
              }
              double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

              NumberFormat formatter = new DecimalFormat("#0.000");
              System.out.println("********** Wheel Radius Characterization Results **********");
              System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
              System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
              System.out.println(
                  "\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                      + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
            })
        .named("Drive:WheelRadiusCharacterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions;
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
  }
}
