// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.spinindexer;

import static org.wpilib.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class SpinIndexerIOTalonFX implements SpinIndexerIO {
  private static final int VELOCITY_SLOT = 0;
  private static final int POSITION_SLOT = 1;

  private final TalonFX motor;
  private final CANcoder cancoder;
  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(VELOCITY_SLOT);
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0.0).withSlot(POSITION_SLOT);
  private final MusicTone musicToneRequest = new MusicTone(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> cancoderPosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorSupplyCurrent;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Current> motorTorqueCurrent;
  private final StatusSignal<AngularVelocity> motorRotorVelocity;
  private final StatusSignal<Temperature> motorTemperature;

  private boolean hardwareConfigurationValid;

  public SpinIndexerIOTalonFX() {
    // TODO: Verify configuration latency with all devices active; writes use 5 retries/0.25 s.
    if (!SpinIndexerConstants.REAL_IO_CONFIGURED) {
      throw new IllegalStateException("SpinIndexer real IO is disabled.");
    }

    CANBus canBus = new CANBus(SpinIndexerConstants.CAN_BUS_NAME);
    motor = new TalonFX(SpinIndexerConstants.MOTOR_ID, canBus);
    cancoder = new CANcoder(SpinIndexerConstants.CANCODER_ID, canBus);

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = SpinIndexerConstants.CANCODER_OFFSET_ROTATIONS;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO: Verify continuous positioning and slot transfer across the +/-0.5 rotation boundary.
    cancoderConfig.MagnetSensor.SensorDirection =
        SpinIndexerConstants.CANCODER_INVERTED == 1
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    boolean cancoderConfigured =
        tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25)).isOK();

    motorConfig.MotorOutput.NeutralMode =
        SpinIndexerConstants.BRAKE_MODE_ENABLED
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted =
        SpinIndexerConstants.MOTOR_INVERTED == 1
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    motorConfig.Feedback.FeedbackRemoteSensorID = SpinIndexerConstants.CANCODER_ID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio =
        SpinIndexerConstants.CANCODER_TO_MECHANISM_RATIO;
    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit =
        SpinIndexerConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0; // Disable sustained-current derating.
    motorConfig.CurrentLimits.StatorCurrentLimit =
        SpinIndexerConstants.STATOR_CURRENT_LIMIT_AMPS;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        SpinIndexerConstants.PEAK_FORWARD_TORQUE_CURRENT_AMPS;
    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        SpinIndexerConstants.PEAK_REVERSE_TORQUE_CURRENT_AMPS;

    motorConfig.Slot0.kP = SpinIndexerConstants.VELOCITY_KP;
    motorConfig.Slot0.kI = SpinIndexerConstants.VELOCITY_KI;
    motorConfig.Slot0.kD = SpinIndexerConstants.VELOCITY_KD;
    motorConfig.Slot0.kS = SpinIndexerConstants.VELOCITY_KS;
    motorConfig.Slot0.kV = SpinIndexerConstants.VELOCITY_KV;
    motorConfig.Slot0.kA = SpinIndexerConstants.VELOCITY_KA;

    motorConfig.Slot1.kP = SpinIndexerConstants.POSITION_KP;
    motorConfig.Slot1.kI = SpinIndexerConstants.POSITION_KI;
    motorConfig.Slot1.kD = SpinIndexerConstants.POSITION_KD;
    motorConfig.Slot1.kS = SpinIndexerConstants.POSITION_KS;
    motorConfig.Slot1.kV = SpinIndexerConstants.POSITION_KV;
    motorConfig.Slot1.kA = SpinIndexerConstants.POSITION_KA;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        SpinIndexerConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        SpinIndexerConstants.MOTION_MAGIC_ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = SpinIndexerConstants.MOTION_MAGIC_JERK;

    hardwareConfigurationValid =
        tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25)).isOK()
            && cancoderConfigured;

    cancoderPosition = cancoder.getPosition();
    cancoderVelocity = cancoder.getVelocity();
    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    motorAppliedVoltage = motor.getMotorVoltage();
    motorSupplyCurrent = motor.getSupplyCurrent();
    motorStatorCurrent = motor.getStatorCurrent();
    motorTorqueCurrent = motor.getTorqueCurrent();
    motorRotorVelocity = motor.getRotorVelocity();
    motorTemperature = motor.getDeviceTemp();

    // TODO: Verify 100 Hz encoder, 50 Hz current/voltage and 4 Hz temperature frames under full CAN load.
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        cancoderPosition,
        cancoderVelocity,
        cancoderAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorAppliedVoltage,
        motorSupplyCurrent,
        motorStatorCurrent,
        motorTorqueCurrent,
        motorRotorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, motorTemperature);
    // RemoteCANcoder needs its position/velocity frames even when no roboRIO code reads them.
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(SpinIndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorAppliedVoltage,
        motorSupplyCurrent,
        motorStatorCurrent,
        motorTorqueCurrent,
        motorTemperature);
    BaseStatusSignal.refreshAll(cancoderPosition, cancoderVelocity, cancoderAbsolutePosition);
    BaseStatusSignal.refreshAll(motorRotorVelocity);

    double externalRotations =
        BaseStatusSignal.getLatencyCompensatedValue(cancoderPosition, cancoderVelocity).in(Rotations)
            / SpinIndexerConstants.CANCODER_TO_MECHANISM_RATIO;
    inputs.hardwareConfigurationValid = hardwareConfigurationValid;
    inputs.position = Rotations.of(externalRotations);
    inputs.velocity = RotationsPerSecond.of(cancoderVelocity.getValue().in(RotationsPerSecond)
        / SpinIndexerConstants.CANCODER_TO_MECHANISM_RATIO);
    inputs.absolutePosition = cancoderAbsolutePosition.getValue();
    inputs.appliedVoltage = motorAppliedVoltage.getValue();
    inputs.supplyCurrent = motorSupplyCurrent.getValue();
    inputs.supplyCurrentAvailable = motorSupplyCurrent.getStatus().isOK();
    inputs.statorCurrent = motorStatorCurrent.getValue();
    inputs.statorCurrentAvailable = motorStatorCurrent.getStatus().isOK();
    inputs.rotorVelocityRadPerSec = motorRotorVelocity.getStatus().isOK()
        ? motorRotorVelocity.getValue().in(RadiansPerSecond) : Double.NaN;
    inputs.torqueCurrent = motorTorqueCurrent.getValue();
    inputs.temperature = motorTemperature.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    if (!hardwareConfigurationValid || !Double.isFinite(voltage.in(Volts))) {
      stop();
      return;
    }
    motor.setControl(voltageRequest.withOutput(Math.clamp(voltage.in(Volts), -12.0, 12.0)));
  }

  @Override
  public void setTorqueCurrent(Current current) {
    if (!hardwareConfigurationValid || !Double.isFinite(current.in(Amps))) {
      stop();
      return;
    }
    motor.setControl(currentRequest.withOutput(current.in(Amps)));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    if (!hardwareConfigurationValid || !Double.isFinite(velocity.in(RotationsPerSecond))) {
      stop();
      return;
    }
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setPosition(Angle position) {
    if (!hardwareConfigurationValid || !Double.isFinite(position.in(Rotations))) {
      stop();
      return;
    }
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public boolean setMusicTone(Frequency frequency) {
    if (!hardwareConfigurationValid) {
      stop();
      return false;
    }
    return motor.setControl(musicToneRequest.withAudioFrequency(frequency)).isOK();
  }

  @Override
  public void stopMusicTone() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void setVelocityPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig.Slot0, 0.25));
  }

  @Override
  public void setPositionPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {
    motorConfig.Slot1.kP = kP;
    motorConfig.Slot1.kI = kI;
    motorConfig.Slot1.kD = kD;
    motorConfig.Slot1.kS = kS;
    motorConfig.Slot1.kV = kV;
    motorConfig.Slot1.kA = kA;
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig.Slot1, 0.25));
  }

  @Override
  public void setMotionMagicConfig(double cruiseVelocity, double acceleration, double jerk) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Math.max(0.0, cruiseVelocity);
    motorConfig.MotionMagic.MotionMagicAcceleration = Math.max(0.0, acceleration);
    motorConfig.MotionMagic.MotionMagicJerk = Math.max(0.0, jerk);
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig.MotionMagic, 0.25));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
