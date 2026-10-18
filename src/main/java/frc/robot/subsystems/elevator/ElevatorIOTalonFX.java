// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.*;

import org.wpilib.driverstation.RobotState;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.mechanisms.*;
import com.ctre.phoenix6.signals.*;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.*;

import frc.robot.constants.Constants;

/** Average = height; Difference = absolute pivot angle. Both motors require Pro on one CANivore. */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final CANcoder pivotEncoder;
  private final DifferentialMechanism<TalonFX> mechanism;
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final StatusSignal<Angle> averagePosition;
  private final StatusSignal<AngularVelocity> averageVelocity;
  private final StatusSignal<Angle> differencePosition;
  private final StatusSignal<AngularVelocity> differenceVelocity;
  private final StatusSignal<Voltage> leaderVoltage, followerVoltage;
  private final StatusSignal<Current> leaderSupply, followerSupply, leaderStator, followerStator;
  private final StatusSignal<AngularVelocity> leaderRotorVelocity, followerRotorVelocity;
  private final StatusSignal<Temperature> leaderTemperature, followerTemperature;

  private final MotionMagicTorqueCurrentFOC heightPositionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final PositionTorqueCurrentFOC pivotPositionRequest =
      new PositionTorqueCurrentFOC(0).withSlot(1);
  private final MusicTone toneRequest = new MusicTone(0);
  private final VoltageOut homingHeightRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage homingPivotRequest =
      new PositionVoltage(0).withSlot(2).withEnableFOC(true);

  private ClosedLoopGains pivotPositionGains = ElevatorConstants.PIVOT_POSITION_GAINS;
  private ControlState controlState = ControlState.STOPPED;
  private TrapezoidProfile pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      ElevatorConstants.PIVOT_PROFILE_VELOCITY_RADIANS_PER_SECOND,
      ElevatorConstants.PIVOT_PROFILE_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
  private TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();
  private double pivotProfileAcceleration;
  private double heightMeters, heightMetersPerSecond, pivotRadians, pivotRadiansPerSecond;
  private double heightGoal, pivotGoal, homingVolts;
  private boolean hardwareConfigurationValid;
  private boolean heightReferenceValid;
  private String controlError = "";

  public ElevatorIOTalonFX() {
    // TODO: Verify configuration latency: startup/runtime writes allow 5 attempts at
    // 0.25/0.05 s each; height reset uses 0.1 s. Confirm retries fit the control timing.
    if (!ElevatorConstants.REAL_IO_CONFIGURED) {
      throw new IllegalStateException("Elevator real IO is disabled.");
    }
    CANBus bus = new CANBus(ElevatorConstants.CAN_BUS_NAME);
    pivotEncoder = new CANcoder(ElevatorConstants.PIVOT_CANCODER_ID, bus);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset =
        ElevatorConstants.PIVOT_CANCODER_OFFSET.in(Rotations);
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ElevatorConstants.PIVOT_CANCODER_DISCONTINUITY;
    encoderConfig.MagnetSensor.SensorDirection = ElevatorConstants.PIVOT_CANCODER_INVERTED == 1
        ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
    boolean encoderConfigured =
        tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(encoderConfig, 0.25)).isOK();

    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.Inverted = ElevatorConstants.LEADER_INVERTED == 1
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = ElevatorConstants.BRAKE_MODE_ENABLED
        ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leaderConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION;
    followerConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0; // Also inherited by the follower.
    leaderConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ElevatorConstants.PEAK_FORWARD_TORQUE_CURRENT_AMPS;
    leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        ElevatorConstants.PEAK_REVERSE_TORQUE_CURRENT_AMPS;
    leaderConfig.Slot0 = heightSlot(ElevatorConstants.HEIGHT_POSITION_GAINS);
    leaderConfig.Slot1 = pivotSlot(pivotPositionGains);
    // Fixed limits on either motor encoder cannot represent the two physical travel limits.
    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    leaderConfig.ClosedLoopGeneral.ContinuousWrap = false;
    leaderConfig.ClosedLoopGeneral.DifferentialContinuousWrap = false;
    leaderConfig.MotionMagic = motionMagicConfig(
        ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY_METERS_PER_SECOND,
        ElevatorConstants.MOTION_MAGIC_ACCELERATION_METERS_PER_SECOND_SQUARED,
        ElevatorConstants.MOTION_MAGIC_JERK_METERS_PER_SECOND_CUBED);

    var motorConstants = new DifferentialMotorConstants<TalonFXConfiguration>()
        .withNetwork(bus)
        .withLeaderId(ElevatorConstants.LEADER_MOTOR_ID)
        .withFollowerId(ElevatorConstants.FOLLOWER_MOTOR_ID)
        .withAlignment(ElevatorConstants.MOTOR_ALIGNMENT == 0
            ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed)
        .withSensorToDifferentialRatio(ElevatorConstants.PIVOT_SENSOR_TO_MECHANISM_RATIO)
        .withClosedLoopRate(ElevatorConstants.CLOSED_LOOP_RATE_HZ)
        .withLeaderInitialConfigs(leaderConfig)
        .withFollowerInitialConfigs(followerConfig);
    mechanism = new DifferentialMechanism<>(TalonFX::new, motorConstants, pivotEncoder);
    leaderMotor = mechanism.getLeader();
    followerMotor = mechanism.getFollower();
    // The constructor mutates these configs but only prints configuration errors.
    // Reapply its resulting configs so failed configuration explicitly inhibits our outputs.
    boolean leaderConfigured =
        tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(leaderConfig, 0.25)).isOK();
    boolean followerConfigured =
        tryUntilOk(5, () -> followerMotor.getConfigurator().apply(followerConfig, 0.25)).isOK();
    hardwareConfigurationValid =
        bus.isNetworkFD() && encoderConfigured && leaderConfigured && followerConfigured;

    averagePosition = mechanism.getAveragePosition();
    averageVelocity = mechanism.getAverageVelocity();
    differencePosition = mechanism.getDifferentialPosition();
    differenceVelocity = mechanism.getDifferentialVelocity();
    leaderVoltage = leaderMotor.getMotorVoltage();
    followerVoltage = followerMotor.getMotorVoltage();
    leaderSupply = leaderMotor.getSupplyCurrent();
    followerSupply = followerMotor.getSupplyCurrent();
    leaderStator = leaderMotor.getStatorCurrent();
    followerStator = followerMotor.getStatorCurrent();
    leaderRotorVelocity = leaderMotor.getRotorVelocity();
    followerRotorVelocity = followerMotor.getRotorVelocity();
    leaderTemperature = leaderMotor.getDeviceTemp();
    followerTemperature = followerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(ElevatorConstants.CLOSED_LOOP_RATE_HZ,
        averagePosition, averageVelocity, differencePosition, differenceVelocity,
        leaderMotor.getPosition(), followerMotor.getPosition(),
        leaderMotor.getVelocity(), followerMotor.getVelocity(),
        pivotEncoder.getPosition(), pivotEncoder.getAbsolutePosition(), pivotEncoder.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(50, leaderVoltage, followerVoltage,
        leaderSupply, followerSupply, leaderStator, followerStator,
        leaderRotorVelocity, followerRotorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(4, leaderTemperature, followerTemperature);
    // Preserve all remote feedback/differential output frames established by the mechanism.
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor, pivotEncoder);
    stop();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(averagePosition, averageVelocity, differencePosition,
        differenceVelocity,
        leaderVoltage, followerVoltage, leaderSupply, followerSupply,
        leaderStator, followerStator, leaderTemperature, followerTemperature);
    // Logging-only signals do not participate in differential motion or homing decisions.
    BaseStatusSignal.refreshAll(leaderRotorVelocity, followerRotorVelocity);
    mechanism.periodic();
    heightMeters = averagePosition.getValue().in(Rotations)
        * ElevatorConstants.metersPerAverageRotation();
    heightMetersPerSecond = averageVelocity.getValue().in(RotationsPerSecond)
        * ElevatorConstants.metersPerAverageRotation();
    pivotRadians = differencePosition.getValue().in(Radians);
    pivotRadiansPerSecond = differenceVelocity.getValue().in(RadiansPerSecond);
    if (mechanism.requiresUserAction()) {
      heightReferenceValid = false;
    }
    if (!canControl() || outsideTravel()
        || (RobotState.isDisabled() && controlState != ControlState.MUSIC)) {
      stop();
    } else if (controlState == ControlState.POSITION || controlState == ControlState.HOMING) {
      if (controlState == ControlState.POSITION) {
        double previousVelocity = pivotSetpoint.velocity;
        pivotSetpoint = pivotProfile.calculate(Constants.loopPeriodSecs, pivotSetpoint,
            new TrapezoidProfile.State(pivotGoal, 0));
        double boundedPosition = Math.clamp(pivotSetpoint.position,
            ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians),
            ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians));
        if (boundedPosition != pivotSetpoint.position) {
          pivotSetpoint = new TrapezoidProfile.State(boundedPosition, 0);
        }
        pivotProfileAcceleration =
            (pivotSetpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
      }
      applyControl();
    }

    inputs.hardwareConfigurationValid = hardwareConfigurationValid;
    inputs.heightReferenceValid = heightReferenceValid;
    inputs.homingActive = controlState == ControlState.HOMING;
    inputs.mechanismFault = mechanism.isDisabled()
        ? mechanism.getDisabledReason().toString()
        : outsideTravel() ? "Physical axis travel exceeded" : controlError;
    inputs.height = Meters.of(heightMeters);
    inputs.heightVelocity = MetersPerSecond.of(heightMetersPerSecond);
    inputs.pivotAngle = Radians.of(pivotRadians);
    inputs.pivotVelocity = RadiansPerSecond.of(pivotRadiansPerSecond);
    inputs.leaderAppliedVoltage = leaderVoltage.getValue();
    inputs.followerAppliedVoltage = followerVoltage.getValue();
    inputs.leaderSupplyCurrent = leaderSupply.getValue();
    inputs.followerSupplyCurrent = followerSupply.getValue();
    inputs.leaderSupplyCurrentAvailable = leaderSupply.getStatus().isOK();
    inputs.followerSupplyCurrentAvailable = followerSupply.getStatus().isOK();
    inputs.leaderStatorCurrent = leaderStator.getValue();
    inputs.followerStatorCurrent = followerStator.getValue();
    inputs.leaderStatorCurrentAvailable = leaderStator.getStatus().isOK();
    inputs.followerStatorCurrentAvailable = followerStator.getStatus().isOK();
    inputs.leaderRotorVelocityRadPerSec = leaderRotorVelocity.getStatus().isOK()
        ? leaderRotorVelocity.getValue().in(RadiansPerSecond) : Double.NaN;
    inputs.followerRotorVelocityRadPerSec = followerRotorVelocity.getStatus().isOK()
        ? followerRotorVelocity.getValue().in(RadiansPerSecond) : Double.NaN;
    inputs.leaderTemperature = leaderTemperature.getValue();
    inputs.followerTemperature = followerTemperature.getValue();
  }

  private boolean canControl() {
    return hardwareConfigurationValid
        && (heightReferenceValid || controlState == ControlState.HOMING)
        && (controlState != ControlState.HOMING || (RobotState.isEnabled()
            && Math.abs(pivotRadians - pivotGoal)
                <= ElevatorConstants.HOMING_MAX_PIVOT_ERROR.in(Radians)))
        && !mechanism.isDisabled() && controlError.isEmpty();
  }

  @Override
  public void setPosition(Distance height, Angle pivot) {
    if (controlState == ControlState.HOMING || !canControl()) { stop(); return; }
    if (controlState != ControlState.POSITION) {
      pivotSetpoint = new TrapezoidProfile.State(pivotRadians, pivotRadiansPerSecond);
      pivotProfileAcceleration = 0;
    }
    controlState = ControlState.POSITION;
    heightGoal = height.in(Meters);
    pivotGoal = pivot.in(Radians);
    applyControl();
  }

  private void applyControl() {
    if (!canControl() || outsideTravel()) { stop(); return; }
    StatusCode status;
    if (controlState == ControlState.HOMING) {
      // Both requests must share an output type; homing alone uses voltage on both axes.
      status = mechanism.setControl(homingHeightRequest.withOutput(homingVolts),
          homingPivotRequest.withPosition(pivotGoal / (2 * Math.PI)).withVelocity(0)
              .withFeedForward(ElevatorConstants.HOMING_PIVOT_VOLTAGE_GAINS.kG()
                  * Math.cos(pivotRadians)));
    } else if (controlState == ControlState.POSITION) {
      // Motion Magic is supported only on Average. Difference follows a host-side profile.
      status = mechanism.setControl(
          heightPositionRequest.withPosition(heightGoal / ElevatorConstants.metersPerAverageRotation()),
          pivotPositionRequest.withPosition(pivotSetpoint.position / (2 * Math.PI))
              .withVelocity(pivotSetpoint.velocity / (2 * Math.PI))
              .withFeedForward(pivotPositionGains.kG() * Math.cos(pivotRadians)
                  + pivotPositionGains.kA() * pivotProfileAcceleration / (2 * Math.PI)));
    } else {
      stop(); return;
    }
    if (!status.isOK()) {
      controlError = status.toString();
      stop();
    }
  }

  private boolean outsideTravel() {
    return pivotRadians < ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians)
            - ElevatorConstants.PIVOT_TOLERANCE.in(Radians)
        || pivotRadians > ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians)
            + ElevatorConstants.PIVOT_TOLERANCE.in(Radians)
        || (heightReferenceValid && (heightMeters < ElevatorConstants.MIN_POSITION.in(Meters)
                - ElevatorConstants.POSITION_TOLERANCE.in(Meters)
            || heightMeters > ElevatorConstants.MAX_POSITION.in(Meters)
                + ElevatorConstants.POSITION_TOLERANCE.in(Meters)));
  }

  @Override
  public boolean resetHeight(Distance height) {
    stop();
    if (!RobotState.isDisabled() || !hardwareConfigurationValid
        || pivotRadians < ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians)
        || pivotRadians > ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians)
        || !Double.isFinite(height.in(Meters))
        || height.in(Meters) < ElevatorConstants.MIN_POSITION.in(Meters)
        || height.in(Meters) > ElevatorConstants.MAX_POSITION.in(Meters)
        || Math.abs(heightMetersPerSecond) > ElevatorConstants.RESET_MAX_HEIGHT_VELOCITY.in(MetersPerSecond)
        || Math.abs(pivotRadiansPerSecond) > ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond)) {
      return false;
    }
    return writeHeightReference(height);
  }

  private boolean writeHeightReference(Distance height) {
    heightReferenceValid = false;
    // Writes both internal sensors. Remote CANcoder feedback is unaffected by this API.
    boolean success = mechanism.setPosition(
        height.in(Meters) / ElevatorConstants.metersPerAverageRotation(), 0, 0.1).isOK();
    if (success) {
      heightMeters = height.in(Meters);
      heightGoal = heightMeters;
      mechanism.clearUserRequirement();
      mechanism.periodic();
      heightReferenceValid = !mechanism.isDisabled();
      controlError = "";
    }
    return heightReferenceValid;
  }

  @Override
  public boolean beginHoming(Angle pivotAngle) {
    stop();
    if (RobotState.isDisabled() || !hardwareConfigurationValid) return false;
    mechanism.periodic();
    if (mechanism.requiresUserAction()) {
      if (mechanism.getRequiresUserReason()
          != DifferentialMechanism.RequiresUserReasonValue.DeviceHasReset) return false;
      // Homing will replace the lost height reference; other native faults stay latched.
      heightReferenceValid = false;
      mechanism.clearUserRequirement();
      mechanism.periodic();
    }
    if (mechanism.isDisabled()) return false;
    ClosedLoopGains gains = ElevatorConstants.HOMING_PIVOT_VOLTAGE_GAINS;
    StatusCode status = tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(
        new Slot2Configs().withKP(gains.kP()).withKI(gains.kI()).withKD(gains.kD()), 0.05));
    controlError = status.isOK() ? "" : status.toString();
    if (!status.isOK()) return false;
    heightReferenceValid = false;
    pivotGoal = pivotAngle.in(Radians);
    controlState = ControlState.HOMING;
    applyControl();
    return controlState == ControlState.HOMING;
  }

  @Override
  public boolean setHomingVoltage(Voltage voltage) {
    if (controlState != ControlState.HOMING || RobotState.isDisabled() || !canControl()
        || !Double.isFinite(voltage.in(Volts)) || voltage.in(Volts) >= 0) {
      stop();
      return false;
    }
    homingVolts = Math.clamp(voltage.in(Volts), ElevatorConstants.HOMING_VOLTAGE.in(Volts), 0);
    applyControl();
    return controlState == ControlState.HOMING;
  }

  @Override
  public boolean completeHoming() {
    if (controlState != ControlState.HOMING || RobotState.isDisabled() || !canControl()
        || Math.abs(heightMetersPerSecond)
            > ElevatorConstants.HOMING_VELOCITY_THRESHOLD.in(MetersPerSecond)
        || Math.abs(pivotRadiansPerSecond)
            > ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond)
        || Math.abs(leaderStator.getValueAsDouble()) < ElevatorConstants.HOMING_CURRENT_THRESHOLD.in(Amps)
        || Math.abs(followerStator.getValueAsDouble()) < ElevatorConstants.HOMING_CURRENT_THRESHOLD.in(Amps)) {
      stop();
      return false;
    }
    boolean success = writeHeightReference(ElevatorConstants.HOME_POSITION);
    stop();
    return success;
  }

  @Override
  public void setPIDGains(Axis axis, ClosedLoopGains gains) {
    if (!gains.isFinite() || gains.kP() <= 0) {
      hardwareConfigurationValid = false;
      stop(); return;
    }
    if (axis == Axis.HEIGHT) {
      hardwareConfigurationValid &= tryUntilOk(5,
          () -> leaderMotor.getConfigurator().apply(heightSlot(gains), 0.05)).isOK();
    } else {
      pivotPositionGains = gains;
      hardwareConfigurationValid &= tryUntilOk(5,
          () -> leaderMotor.getConfigurator().apply(pivotSlot(gains), 0.05)).isOK();
    }
    if (!hardwareConfigurationValid) stop();
  }
  @Override
  public void setMotionMagicConfig(double cruise, double acceleration, double jerk) {
    if (!Double.isFinite(cruise) || cruise <= 0 || !Double.isFinite(acceleration)
        || acceleration <= 0 || !Double.isFinite(jerk) || jerk < 0) {
      hardwareConfigurationValid = false; stop(); return;
    }
    hardwareConfigurationValid &= tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(
        motionMagicConfig(cruise, acceleration, jerk), 0.05)).isOK();
    if (!hardwareConfigurationValid) stop();
  }
  @Override
  public void setPivotProfileConfig(double velocity, double acceleration) {
    if (!Double.isFinite(velocity) || velocity <= 0
        || !Double.isFinite(acceleration) || acceleration <= 0) {
      hardwareConfigurationValid = false; stop(); return;
    }
    pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(velocity, acceleration));
  }

  private static Slot0Configs heightSlot(ClosedLoopGains g) {
    return new Slot0Configs().withKP(g.kP()).withKI(g.kI()).withKD(g.kD())
        .withKS(g.kS()).withKV(g.kV()).withKA(g.kA()).withKG(g.kG())
        .withGravityType(GravityTypeValue.Elevator_Static);
  }
  private static Slot1Configs pivotSlot(ClosedLoopGains g) {
    // Supply cosine gravity explicitly from the measured Difference axis angle.
    return new Slot1Configs().withKP(g.kP()).withKI(g.kI()).withKD(g.kD())
        .withKS(g.kS()).withKV(g.kV()).withKA(0).withKG(0);
  }
  private static MotionMagicConfigs motionMagicConfig(double cruise, double acceleration, double jerk) {
    double scale = ElevatorConstants.metersPerAverageRotation();
    return new MotionMagicConfigs().withMotionMagicCruiseVelocity(cruise / scale)
        .withMotionMagicAcceleration(acceleration / scale).withMotionMagicJerk(jerk / scale);
  }

  @Override
  public boolean setMusicTone(Frequency frequency) {
    stop();
    if (!canControl() || outsideTravel() || !RobotState.isDisabled()
        || !Double.isFinite(frequency.in(Hertz)) || frequency.in(Hertz) < 0) return false;
    boolean first = leaderMotor.setControl(toneRequest.withAudioFrequency(frequency)).isOK();
    boolean second = followerMotor.setControl(toneRequest.withAudioFrequency(frequency)).isOK();
    if (!first || !second) { stop(); return false; }
    controlState = ControlState.MUSIC;
    return true;
  }
  @Override
  public void stopMusicTone() { stop(); }
  @Override
  public void setBrakeModeEnabled(boolean enabled) {
    hardwareConfigurationValid &= tryUntilOk(5, () -> mechanism.configNeutralMode(
        enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.05)).isOK();
    if (!hardwareConfigurationValid) stop();
  }
  @Override
  public void stop() {
    controlState = ControlState.STOPPED;
    homingVolts = 0;
    // Direct neutral requests also stop music and work when the differential helper is faulted.
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }
}
