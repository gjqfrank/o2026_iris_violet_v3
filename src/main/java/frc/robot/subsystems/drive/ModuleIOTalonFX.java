// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  // At 50 Hz, allow five missed velocity frames before rejecting coupling feedback.
  private static final double COUPLING_FEEDBACK_MAX_AGE_SECONDS = 0.1; // TODO: Verify feedback-loss stop timing and healthy CAN latency, s.

  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;
  private final double couplingWheelRotationsPerAzimuthRotation;
  private double couplingPositionReferenceRot = Double.NaN;
  private double lastValidDrivePositionRot = 0.0;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;
  private final TalonFXConfiguration driveConfig;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final MusicTone driveMusicToneRequest = new MusicTone(0.0);
  private final MusicTone turnMusicToneRequest = new MusicTone(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<AngularVelocity> driveRotorVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<AngularVelocity> turnRotorVelocity;
  // Independent continuous CANcoder feedback keeps Talon setPosition from changing wheel distance.
  private final StatusSignal<Angle> couplingPosition;
  private final Queue<Double> couplingPositionQueue;
  private final StatusSignal<AngularVelocity> couplingVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrent;
  private final StatusSignal<Current> turnStatorCurrent;

  // Connection debouncers
  // TODO: Validate the 0.5 s falling debounce for connection Alerts; coupling output protection uses its own age check.
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    // TODO: Verify configuration latency with the full CAN bus: startup uses 5 retries/0.25 s,
    // runtime gain/current-limit writes use 5 retries/0.05 s.
    // getPosition/getVelocity already include SensorToMechanismRatio; C is in rotor rotations.
    couplingWheelRotationsPerAzimuthRotation =
        constants.CouplingGearRatio / constants.DriveMotorGearRatio;
    driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

    // Configure drive motor
    driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.SUPPLY_LIMIT_AUTO;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.SUPPLY_LIMIT_AUTO;
    driveConfig.Audio.AllowMusicDurDisable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    // Preserve configured current limits without sharing a mutable config group.
    turnConfig.CurrentLimits = constants.SteerMotorInitialConfigs.CurrentLimits.clone();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default ->
              throw new RuntimeException(
                  "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Audio.AllowMusicDurDisable = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    var odometryThread = PhoenixOdometryThread.getInstance();
    timestampQueue = odometryThread.makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue = odometryThread.registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity();
    driveRotorVelocity = driveTalon.getRotorVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = odometryThread.registerSignal(turnPosition.clone());
    turnVelocity = turnTalon.getVelocity();
    turnRotorVelocity = turnTalon.getRotorVelocity();
    couplingPosition = cancoder.getPosition();
    couplingPositionQueue = odometryThread.registerSignal(couplingPosition.clone());
    couplingVelocity = cancoder.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnStatorCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    // TODO: Verify 50 Hz velocity/current telemetry remains fresh under the configured odometry CAN load.
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition, couplingPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveRotorVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveStatorCurrent,
        driveTorqueCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnRotorVelocity,
        couplingVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnStatorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveStatorCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(
            turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnStatorCurrent);
    var turnEncoderStatus =
        BaseStatusSignal.refreshAll(turnAbsolutePosition, couplingPosition, couplingVelocity);
    // Energy telemetry is refreshed separately so it cannot alter the existing motion-health gate.
    BaseStatusSignal.refreshAll(driveRotorVelocity, turnRotorVelocity);
    inputs.motionFeedbackAvailable = driveStatus.isOK() && turnStatus.isOK()
        && turnEncoderStatus.isOK();
    inputs.motionFeedbackAgeSeconds = 0.0;
    for (var signal : new BaseStatusSignal[] {
        drivePosition, driveVelocity, turnPosition, turnVelocity,
        turnAbsolutePosition, couplingPosition, couplingVelocity}) {
      double age = signal.getTimestamp().getLatency();
      if (!Double.isFinite(age) || age < 0) inputs.motionFeedbackAvailable = false;
      inputs.motionFeedbackAgeSeconds = Math.max(inputs.motionFeedbackAgeSeconds, age);
    }

    // All three queues are captured under the same odometry lock and use the same sample indices.
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    double[] rawDrivePositionsRot =
        drivePositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
    double[] couplingPositionsRot =
        couplingPositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    // Anchor the azimuth reference once so an arbitrary startup angle is not treated as wheel travel.
    if (Double.isNaN(couplingPositionReferenceRot) && couplingFeedbackValid()) {
      couplingPositionReferenceRot =
          couplingPositionsRot.length > 0 && Double.isFinite(couplingPositionsRot[0])
              ? couplingPositionsRot[0]
              : couplingPosition.getValueAsDouble();
    }

    // Process the paired samples chronologically before the current position snapshot.
    inputs.odometryDrivePositionsRot = new double[rawDrivePositionsRot.length];
    for (int i = 0; i < rawDrivePositionsRot.length; i++) {
      inputs.odometryDrivePositionsRot[i] =
          updateCompensatedDrivePosition(rawDrivePositionsRot[i], couplingPositionsRot[i]);
    }

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRot =
        updateCompensatedDrivePosition(
            drivePosition.getValueAsDouble(), couplingPosition.getValueAsDouble());
    inputs.driveVelocityRotPerSec =
        driveVelocity.getValueAsDouble()
            - couplingWheelRotationsPerAzimuthRotation * couplingVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveRotorVelocityRadPerSec = driveRotorVelocity.getValue().in(RadiansPerSecond);
    inputs.driveEnergyTelemetryAvailable = driveSupplyCurrent.getStatus().isOK()
        && driveStatorCurrent.getStatus().isOK() && driveRotorVelocity.getStatus().isOK();
    // Characterization telemetry does not participate in the chassis feedback-health gate.
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.refresh().getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRotPerSec = turnVelocity.getValueAsDouble();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnRotorVelocityRadPerSec = turnRotorVelocity.getValue().in(RadiansPerSecond);
    inputs.turnEnergyTelemetryAvailable = turnSupplyCurrent.getStatus().isOK()
        && turnStatorCurrent.getStatus().isOK() && turnRotorVelocity.getStatus().isOK();

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
    couplingPositionQueue.clear();
  }

  private double updateCompensatedDrivePosition(double drivePositionRot, double azimuthPositionRot) {
    if (!Double.isFinite(couplingPositionReferenceRot)
        || !Double.isFinite(drivePositionRot) || !Double.isFinite(azimuthPositionRot)) {
      return lastValidDrivePositionRot;
    }
    lastValidDrivePositionRot = drivePositionRot
        - couplingWheelRotationsPerAzimuthRotation
            * (azimuthPositionRot - couplingPositionReferenceRot);
    return lastValidDrivePositionRot;
  }

  private boolean couplingFeedbackValid() {
    double positionAgeSeconds = couplingPosition.getTimestamp().getLatency();
    double velocityAgeSeconds = couplingVelocity.getTimestamp().getLatency();
    return couplingPosition.getStatus().isOK() && couplingVelocity.getStatus().isOK()
        && Double.isFinite(couplingPosition.getValueAsDouble())
        && Double.isFinite(couplingVelocity.getValueAsDouble())
        && Double.isFinite(positionAgeSeconds) && positionAgeSeconds >= 0.0
        && positionAgeSeconds <= COUPLING_FEEDBACK_MAX_AGE_SECONDS
        && Double.isFinite(velocityAgeSeconds) && velocityAgeSeconds >= 0.0
        && velocityAgeSeconds <= COUPLING_FEEDBACK_MAX_AGE_SECONDS;
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveOpenLoopCurrent(double outputAmps) {
    driveTalon.setControl(torqueCurrentRequest.withOutput(outputAmps));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforwardAmps) {
    if (!couplingFeedbackValid()) {
      // Never add an old azimuth velocity to a wheel-speed request after CANcoder feedback is lost.
      driveTalon.setControl(neutralRequest);
      return;
    }
    double velocityRotPerSec =
        Units.radiansToRotations(velocityRadPerSec)
            + couplingWheelRotationsPerAzimuthRotation * couplingVelocity.getValueAsDouble();
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest
                  .withVelocity(velocityRotPerSec)
                  .withFeedForward(feedforwardAmps);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    setDriveVelocity(velocityRadPerSec, 0.0);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }

  @Override
  public boolean setMusicTone(Frequency frequency) {
    boolean succeeded =
        driveTalon.setControl(driveMusicToneRequest.withAudioFrequency(frequency)).isOK();
    succeeded &= turnTalon.setControl(turnMusicToneRequest.withAudioFrequency(frequency)).isOK();
    return succeeded;
  }

  @Override
  public void stopMusicTone() {
    driveTalon.setControl(neutralRequest);
    turnTalon.setControl(neutralRequest);
  }

  @Override
  public void updateDrivePIDGains(
      double kP, double kI, double kD, double kV, double kS, double kA) {
    var slot0 =
        constants.DriveMotorGains.clone()
            .withKP(kP).withKI(kI).withKD(kD).withKV(kV).withKS(kS).withKA(kA);
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(slot0, 0.05));
  }

  @Override
  public void updateTurnPIDGains(double kP, double kI, double kD, double kV, double kS, double kA) {
    var slot0 =
        constants.SteerMotorGains.clone()
            .withKP(kP).withKI(kI).withKD(kD).withKV(kV).withKS(kS).withKA(kA);
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(slot0, 0.05));
  }

  @Override
  public void updateStatorCurrentLimit(double limit) {
    driveConfig.CurrentLimits.StatorCurrentLimit = limit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    applyCurrentLimits();
  }

  @Override
  public void updateSupplyCurrentLimit(double limit) {
    driveConfig.CurrentLimits.SupplyCurrentLimit = limit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit = limit;
    applyCurrentLimits();
  }

  private void applyCurrentLimits() {
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig.CurrentLimits, 0.05));
  }

  @Override
  public void resetAbsoluteTurnPosition() {
    tryUntilOk(
        5, () -> turnTalon.setPosition(cancoder.getAbsolutePosition().getValueAsDouble(), 0.25));
  }
}
