// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.spinindexer;

import static org.wpilib.units.Units.*;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.subsystems.common.ControlMode;

public class SpinIndexerIOSim implements SpinIndexerIO {
  private final DCMotorSim sim;
  private final PIDController velocityController =
      new PIDController(
          SpinIndexerConstants.SIM_VELOCITY_KP,
          SpinIndexerConstants.SIM_VELOCITY_KI,
          SpinIndexerConstants.SIM_VELOCITY_KD);
  private final PIDController positionController =
      new PIDController(
          SpinIndexerConstants.SIM_POSITION_KP,
          SpinIndexerConstants.SIM_POSITION_KI,
          SpinIndexerConstants.SIM_POSITION_KD);

  private double velocityKS = SpinIndexerConstants.SIM_VELOCITY_KS;
  private double velocityKV = SpinIndexerConstants.SIM_VELOCITY_KV;
  private double velocityKA = SpinIndexerConstants.SIM_VELOCITY_KA;
  private double positionKS = SpinIndexerConstants.SIM_POSITION_KS;
  private double positionKV = SpinIndexerConstants.SIM_POSITION_KV;
  private double positionKA = SpinIndexerConstants.SIM_POSITION_KA;
  private double motionMagicCruiseVelocity =
      SpinIndexerConstants.MOTION_MAGIC_CRUISE_VELOCITY;
  private double motionMagicAcceleration = SpinIndexerConstants.MOTION_MAGIC_ACCELERATION;
  private double motionMagicJerk = SpinIndexerConstants.MOTION_MAGIC_JERK;

  private Voltage appliedVoltage = Volts.of(0.0);
  private double targetCurrentAmps = 0.0;
  private AngularVelocity targetVelocity = RotationsPerSecond.of(0.0);
  private Angle targetPosition = Rotations.of(0.0);
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private double profileAcceleration = 0.0;
  private double previousTargetVelocity = 0.0;
  private ControlMode controlMode = ControlMode.STOPPED;

  public SpinIndexerIOSim() {
    sim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                SpinIndexerConstants.SIM_MOTOR,
                SpinIndexerConstants.SIM_MOI.in(KilogramSquareMeters),
                SpinIndexerConstants.SIM_REDUCTION),
            SpinIndexerConstants.SIM_MOTOR);
  }

  @Override
  public void updateInputs(SpinIndexerIOInputs inputs) {
    switch (controlMode) {
      case VELOCITY -> updateVelocityControl();
      case POSITION -> updatePositionControl();
      case TORQUE_CURRENT -> applyTorqueCurrent(targetCurrentAmps);
      case VOLTAGE, STOPPED -> {}
      default -> stop();
    }

    if (RobotState.isDisabled()) {
      appliedVoltage = Volts.of(0.0);
    }

    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(Constants.loopPeriodSecs);

    double positionRotations = org.wpilib.math.util.Units.radiansToRotations(sim.getAngularPosition());
    double absolutePositionRotations = MathUtil.inputModulus(positionRotations, -0.5, 0.5);
    double motorVelocityRadPerSec =
      sim.getAngularVelocity() * SpinIndexerConstants.SIM_REDUCTION;
    // DCMotorSim clamps the requested voltage to the simulated battery voltage.
    double actualVoltage = sim.getInputVoltage();

    inputs.hardwareConfigurationValid = true;
    inputs.position = Rotations.of(positionRotations);
    inputs.absolutePosition = Rotations.of(absolutePositionRotations);
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocity());
    inputs.appliedVoltage = Volts.of(actualVoltage);
    double windingCurrent = SpinIndexerConstants.SIM_MOTOR.getCurrent(
        motorVelocityRadPerSec, actualVoltage);
    inputs.rotorVelocityRadPerSec = motorVelocityRadPerSec;
    inputs.statorCurrentAvailable = true;
    inputs.statorCurrent = Amps.of(windingCurrent * motorVelocityRadPerSec < 0
        ? -Math.abs(windingCurrent) : Math.abs(windingCurrent));
    // SysId retains direction-signed torque current, independent of the energy sign convention.
    inputs.torqueCurrent = Amps.of(windingCurrent);
    inputs.supplyCurrentAvailable = true;
    inputs.supplyCurrent = Amps.of(inputs.torqueCurrent.in(Amps) * actualVoltage
        / RobotController.getBatteryVoltage());
  }

  private void updateVelocityControl() {
    double currentVelocity = sim.getAngularVelocity() / (2 * Math.PI);
    double targetRotationsPerSecond = targetVelocity.in(RotationsPerSecond);
    double targetAcceleration =
        (targetRotationsPerSecond - previousTargetVelocity) / Constants.loopPeriodSecs;
    double feedbackCurrent =
        velocityController.calculate(currentVelocity, targetRotationsPerSecond);
    double feedforwardCurrent =
        calculateFeedforwardCurrent(
            velocityKS, velocityKV, velocityKA, targetRotationsPerSecond, targetAcceleration);
    applyTorqueCurrent(feedbackCurrent + feedforwardCurrent);
    previousTargetVelocity = targetRotationsPerSecond;
  }

  private void updatePositionControl() {
    double currentPosition = org.wpilib.math.util.Units.radiansToRotations(sim.getAngularPosition());
    double nearestTarget = profileState.position + MathUtil.inputModulus(targetPosition.in(Rotations) - profileState.position, -0.5, 0.5);
    TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Math.max(motionMagicCruiseVelocity, 1.0e-6),
                Math.max(motionMagicAcceleration, 1.0e-6)));
    TrapezoidProfile.State nextProfileState =
        profile.calculate(
            Constants.loopPeriodSecs,
            profileState,
            new TrapezoidProfile.State(nearestTarget, 0.0));

    double requestedAcceleration =
        (nextProfileState.velocity - profileState.velocity) / Constants.loopPeriodSecs;
    double limitedAcceleration = requestedAcceleration;
    if (motionMagicJerk > 0.0) {
      double maximumAccelerationChange = motionMagicJerk * Constants.loopPeriodSecs;
      limitedAcceleration =
          Math.clamp(
              requestedAcceleration,
              profileAcceleration - maximumAccelerationChange,
              profileAcceleration + maximumAccelerationChange);
    }

    double limitedVelocity =
      Math.clamp(profileState.velocity + limitedAcceleration * Constants.loopPeriodSecs, -motionMagicCruiseVelocity, motionMagicCruiseVelocity);
    double limitedPosition =
        profileState.position
            + (profileState.velocity + limitedVelocity) * 0.5 * Constants.loopPeriodSecs;
    if ((nearestTarget - profileState.position) * (nearestTarget - limitedPosition) <= 0.0) {
      limitedPosition = nearestTarget;
      limitedVelocity = 0.0;
      limitedAcceleration = 0.0;
    }
    profileState = new TrapezoidProfile.State(limitedPosition, limitedVelocity);
    profileAcceleration = limitedAcceleration;

    double feedbackCurrent = positionController.calculate(currentPosition, profileState.position);
    double feedforwardCurrent =
        calculateFeedforwardCurrent(
            positionKS, positionKV, positionKA, profileState.velocity, profileAcceleration);
    applyTorqueCurrent(feedbackCurrent + feedforwardCurrent);
  }

  private static double calculateFeedforwardCurrent(
      double kS, double kV, double kA, double velocity, double acceleration) {
    double staticCurrent = Math.abs(velocity) > 1.0e-6 ? Math.copySign(kS, velocity) : 0.0;
    return staticCurrent + kV * velocity + kA * acceleration;
  }

  private void applyTorqueCurrent(double requestedCurrentAmps) {
    double clampedCurrent =
        Math.clamp(
            requestedCurrentAmps,
            SpinIndexerConstants.PEAK_REVERSE_TORQUE_CURRENT_AMPS,
            SpinIndexerConstants.PEAK_FORWARD_TORQUE_CURRENT_AMPS);
    double motorVelocityRadPerSec =
        sim.getAngularVelocity() * SpinIndexerConstants.SIM_REDUCTION;
    double requestedVoltage =
        SpinIndexerConstants.SIM_MOTOR.getVoltage(
            SpinIndexerConstants.SIM_MOTOR.getTorque(clampedCurrent), motorVelocityRadPerSec);
    appliedVoltage = Volts.of(Math.clamp(requestedVoltage, -12.0, 12.0));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    controlMode = ControlMode.VOLTAGE;
    resetClosedLoopControllers();
    appliedVoltage = Volts.of(Math.clamp(voltage.in(Volts), -12.0, 12.0));
  }

  @Override
  public void setTorqueCurrent(Current current) {
    controlMode = ControlMode.TORQUE_CURRENT;
    resetClosedLoopControllers();
    targetCurrentAmps = current.in(Amps);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    if (controlMode != ControlMode.VELOCITY) {
      resetClosedLoopControllers();
      previousTargetVelocity = sim.getAngularVelocity() / (2 * Math.PI);
    }
    controlMode = ControlMode.VELOCITY;
    targetVelocity = velocity;
  }

  @Override
  public void setPosition(Angle position) {
    if (controlMode != ControlMode.POSITION) {
      resetClosedLoopControllers();
      profileState =
          new TrapezoidProfile.State(
              org.wpilib.math.util.Units.radiansToRotations(sim.getAngularPosition()), sim.getAngularVelocity() / (2 * Math.PI));
      profileAcceleration = 0.0;
    }
    controlMode = ControlMode.POSITION;
    targetPosition = position;
  }

  @Override
  public void setVelocityPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {
    velocityController.setPID(kP, kI, kD);
    velocityKS = kS;
    velocityKV = kV;
    velocityKA = kA;
  }

  @Override
  public void setPositionPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {
    positionController.setPID(kP, kI, kD);
    positionKS = kS;
    positionKV = kV;
    positionKA = kA;
  }

  @Override
  public void setMotionMagicConfig(double cruiseVelocity, double acceleration, double jerk) {
    motionMagicCruiseVelocity = Math.max(0.0, cruiseVelocity);
    motionMagicAcceleration = Math.max(0.0, acceleration);
    motionMagicJerk = Math.max(0.0, jerk);
  }

  @Override
  public void stop() {
    controlMode = ControlMode.STOPPED;
    targetVelocity = RotationsPerSecond.of(0.0);
    targetPosition = Rotations.of(org.wpilib.math.util.Units.radiansToRotations(sim.getAngularPosition()));
    appliedVoltage = Volts.of(0.0);
    resetClosedLoopControllers();
  }

  private void resetClosedLoopControllers() {
    velocityController.reset();
    positionController.reset();
    profileAcceleration = 0.0;
    previousTargetVelocity = 0.0;
  }
}
