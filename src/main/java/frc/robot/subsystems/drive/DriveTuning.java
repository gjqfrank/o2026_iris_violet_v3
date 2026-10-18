// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;

/** Enable Constants.tuningMode before startup to edit these /Tuning/Drive dashboard values. */
public final class DriveTuning {
  public static final LoggedTunableNumber DEADBAND =
      new LoggedTunableNumber("Drive/Deadband", DriveConstants.DEADBAND);
  public static final LoggedTunableNumber ANGLE_KP =
      new LoggedTunableNumber("Drive/AngleKp", DriveConstants.ANGLE_KP);
  public static final LoggedTunableNumber ANGLE_KD =
      new LoggedTunableNumber("Drive/AngleKd", DriveConstants.ANGLE_KD);
  public static final LoggedTunableNumber ANGLE_MAX_VELOCITY =
      new LoggedTunableNumber(
          "Drive/AngleMaxVelocityRadPerSec",
          DriveConstants.ANGLE_MAX_VELOCITY.in(RadiansPerSecond));
  public static final LoggedTunableNumber ANGLE_MAX_ACCELERATION =
      new LoggedTunableNumber(
          "Drive/AngleMaxAccelerationRadPerSecPerSec",
          DriveConstants.ANGLE_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

  // Phoenix input units are wheel rotations/s for drive and azimuth rotations for steer.
  // Gain output units follow the motor mode (currently torque current in amps).
  public static final LoggedTunableNumber DRIVE_KP =
      new LoggedTunableNumber("Drive/Module/DriveKP", TunerConstants.DRIVE_KP);
  public static final LoggedTunableNumber DRIVE_KI =
      new LoggedTunableNumber("Drive/Module/DriveKI", TunerConstants.DRIVE_KI);
  public static final LoggedTunableNumber DRIVE_KD =
      new LoggedTunableNumber("Drive/Module/DriveKD", TunerConstants.DRIVE_KD);
  public static final LoggedTunableNumber DRIVE_KV =
      new LoggedTunableNumber("Drive/Module/DriveKV", TunerConstants.DRIVE_KV);
  public static final LoggedTunableNumber DRIVE_KS =
      new LoggedTunableNumber("Drive/Module/DriveKS", TunerConstants.DRIVE_KS);
  public static final LoggedTunableNumber DRIVE_KA =
      new LoggedTunableNumber("Drive/Module/DriveKA", TunerConstants.DRIVE_KA);
  public static final LoggedTunableNumber STEER_KP =
      new LoggedTunableNumber("Drive/Module/SteerKP", TunerConstants.STEER_KP);
  public static final LoggedTunableNumber STEER_KI =
      new LoggedTunableNumber("Drive/Module/SteerKI", TunerConstants.STEER_KI);
  public static final LoggedTunableNumber STEER_KD =
      new LoggedTunableNumber("Drive/Module/SteerKD", TunerConstants.STEER_KD);
  public static final LoggedTunableNumber STEER_KV =
      new LoggedTunableNumber("Drive/Module/SteerKV", TunerConstants.STEER_KV);
  public static final LoggedTunableNumber STEER_KS =
      new LoggedTunableNumber("Drive/Module/SteerKS", TunerConstants.STEER_KS);
  public static final LoggedTunableNumber STEER_KA =
      new LoggedTunableNumber("Drive/Module/SteerKA", TunerConstants.STEER_KA);

  // PathPlanner gain tunables were removed with the PathPlanner wiring (Commands v3 migration).

  private DriveTuning() {}

  public static double deadband() {
    double value = DEADBAND.get();
    return Double.isFinite(value) && value >= 0 && value < 1 ? value : DriveConstants.DEADBAND;
  }
}
