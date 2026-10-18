// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Hertz;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Frequency;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public boolean motionFeedbackAvailable = false;
    public double motionFeedbackAgeSeconds = Double.NaN;
    // Physical wheel rotations and rotations/s, with any azimuth coupling removed by the IO.
    public double drivePositionRot = 0.0;
    public double driveVelocityRotPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    // Stator current is positive for motoring and negative for regeneration, regardless of direction.
    public double driveStatorCurrentAmps = 0.0;
    // Signed measured torque current; positive follows the configured forward motor direction.
    public double driveTorqueCurrentAmps = 0.0;
    // Energy analysis uses raw rotor speed, without feedback scaling or azimuth compensation.
    public boolean driveEnergyTelemetryAvailable = false;
    public double driveRotorVelocityRadPerSec = Double.NaN;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.ZERO;
    public Rotation2d turnPosition = Rotation2d.ZERO;
    public double turnVelocityRotPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    // Same direction-independent motoring/regeneration convention as the drive motor.
    public double turnStatorCurrentAmps = 0.0;
    public boolean turnEnergyTelemetryAvailable = false;
    public double turnRotorVelocityRadPerSec = Double.NaN;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRot = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  default void setDriveOpenLoop(double output) {}

  /** Run the drive motor at the specified open loop current. */
  default void setDriveOpenLoopCurrent(double outputAmps) {}

  /** Run the turn motor at the specified open loop value. */
  default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocityRadPerSec, double feedforwardAmps) {}

  /** Run the turn motor to the specified rotation. */
  default void setTurnPosition(Rotation2d rotation) {}

  /** Plays a music tone through the drive and turn motors. */
  default boolean setMusicTone(Frequency frequency) {
    return false;
  }

  /** Stops music output from the drive and turn motors. */
  default void stopMusicTone() {
    setMusicTone(Hertz.zero());
  }

  /** Update PID/FF gains for the drive motor. */
  default void updateDrivePIDGains(
      double kP, double kI, double kD, double kV, double kS, double kA) {}

  /** Update PID/FF gains for the turn motor. */
  default void updateTurnPIDGains(
      double kP, double kI, double kD, double kV, double kS, double kA) {}

  default void updateSupplyCurrentLimit(double limit) {}

  default void updateStatorCurrentLimit(double limit) {}

  /** Reset the turn motor position to the absolute encoder position. */
  default void resetAbsoluteTurnPosition() {}
}
