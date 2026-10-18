// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Frequency;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import frc.robot.energy.BatteryChannel;
import frc.robot.energy.BatteryLogger;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private double lastFeedbackTimestamp = Double.NaN;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert("motor disconnected","Disconnected drive motor on module " + index + ".", Level.HIGH);
    turnDisconnectedAlert =
        new Alert("motor disconnected","Disconnected turn motor on module " + index + ".", Level.HIGH);
    turnEncoderDisconnectedAlert =
        new Alert("encoder disconnected","Disconnected turn encoder on module " + index + ".", Level.HIGH);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    if (sampleCount > 0) lastFeedbackTimestamp = inputs.odometryTimestamps[sampleCount - 1];
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          Units.rotationsToRadians(inputs.odometryDrivePositionsRot[i]) * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    // Report battery load through the explicit per-module channels.
    BatteryLogger.getInstance()
        .reportSupplyCurrent(
            BatteryChannel.driveChannel(index),
            inputs.driveEnergyTelemetryAvailable ? inputs.driveSupplyCurrentAmps : 0.0);
    BatteryLogger.getInstance()
        .reportSupplyCurrent(
            BatteryChannel.turnChannel(index),
            inputs.turnEnergyTelemetryAvailable ? inputs.turnSupplyCurrentAmps : 0.0);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public double runSetpoint(SwerveModuleVelocity state, double driveFeedforwardAmps) {
    double optimizedFeedforwardAmps = driveFeedforwardAmps * optimizeState(state);

    // Apply setpoints
    io.setDriveVelocity(
        state.velocity / constants.WheelRadius, optimizedFeedforwardAmps);
    io.setTurnPosition(state.angle);
    return optimizedFeedforwardAmps;
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleVelocity state) {
    optimizeState(state);

    // Apply setpoints
    io.setDriveVelocity(state.velocity / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  private double optimizeState(SwerveModuleVelocity state) {
    boolean reversesDriveDirection =
        Math.abs(state.angle.minus(inputs.turnPosition).getDegrees()) > 90.0;
    state = state.optimize(inputs.turnPosition);
    double cosineScale = state.angle.minus(inputs.turnPosition).getCos();
    state = state.cosineScale(inputs.turnPosition);
    return reversesDriveDirection ? -cosineScale : cosineScale;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.ZERO);
  }

  /** Runs drive characterization with the specified torque current while holding zero degrees. */
  public void runCurrentCharacterization(double outputAmps) {
    io.setDriveOpenLoopCurrent(outputAmps);
    io.setTurnPosition(Rotation2d.ZERO);
  }

  /** Runs the module with the specified turn position. */
  public void runTurnPosition(Rotation2d turnPosition) {
    io.setTurnPosition(turnPosition);
  }

  /** Plays a music tone through both motors in the module. */
  public boolean setMusicTone(Frequency frequency) {
    return io.setMusicTone(frequency);
  }

  /** Stops music output and neutralizes both motors in the module. */
  public void stopMusicTone() {
    io.stopMusicTone();
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return Units.rotationsToRadians(inputs.drivePositionRot) * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return Units.rotationsToRadians(inputs.driveVelocityRotPerSec) * constants.WheelRadius;
  }

  /** Disconnected or stale module feedback must never count as a stationary chassis. */
  public boolean feedbackHealthy(double now, double maxAgeSeconds) {
    double age = now - lastFeedbackTimestamp;
    return inputs.driveConnected && inputs.turnConnected && inputs.turnEncoderConnected
        && inputs.motionFeedbackAvailable && Double.isFinite(inputs.motionFeedbackAgeSeconds)
        && inputs.motionFeedbackAgeSeconds >= 0 && inputs.motionFeedbackAgeSeconds <= maxAgeSeconds
        && Double.isFinite(age) && age >= 0 && age <= maxAgeSeconds
        && Double.isFinite(getVelocityMetersPerSec())
        && Double.isFinite(inputs.turnPosition.getRadians());
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return Units.rotationsToRadians(inputs.drivePositionRot);
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRotPerSec;
  }

  /** Reset turn motor position to the absolute encoder position. */
  public void resetAbsoluteTurnPosition() {
    io.resetAbsoluteTurnPosition();
  }

  public void updateSupplyCurrentLimit(double limit) {
    io.updateSupplyCurrentLimit(limit);
  }

  public void updateStatorCurrentLimit(double limit) {
    io.updateStatorCurrentLimit(limit);
  }

  /** Applies changed real-motor gains outside the odometry lock. */
  void updateTuning() {
    LoggedTunableNumber.ifChanged(hashCode(), gains ->
        io.updateDrivePIDGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]),
        DriveTuning.DRIVE_KP, DriveTuning.DRIVE_KI, DriveTuning.DRIVE_KD,
        DriveTuning.DRIVE_KV, DriveTuning.DRIVE_KS, DriveTuning.DRIVE_KA);
    LoggedTunableNumber.ifChanged(hashCode(), gains ->
        io.updateTurnPIDGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]),
        DriveTuning.STEER_KP, DriveTuning.STEER_KI, DriveTuning.STEER_KD,
        DriveTuning.STEER_KV, DriveTuning.STEER_KS, DriveTuning.STEER_KA);
  }
}
