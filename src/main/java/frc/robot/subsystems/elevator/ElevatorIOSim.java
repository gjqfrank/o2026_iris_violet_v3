// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.*;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.*;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;

/**
 * Two ideal mechanism plants share two motors' voltage/current headroom.
 * Belt compliance, backlash, and changes in pivot dynamics during lift acceleration are not modeled.
 */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim heightSim;
  private final SingleJointedArmSim pivotSim;
  private final DCMotor singleMotor = DCMotor.getKrakenX60Foc(1);
  private final PIDController heightPositionPID = new PIDController(0, 0, 0);
  private final PIDController pivotPositionPID = new PIDController(0, 0, 0);
  private final PIDController homingPivotPID = new PIDController(0, 0, 0);
  private ClosedLoopGains heightPositionGains = ElevatorConstants.SIM_HEIGHT_POSITION_GAINS;
  private ClosedLoopGains pivotPositionGains = ElevatorConstants.SIM_PIVOT_POSITION_GAINS;
  private ControlState controlState = ControlState.STOPPED;
  private double heightGoal, pivotGoal, homingVolts;
  private double leaderVolts, followerVolts, leaderAmps, followerAmps;
  private double heightCruise = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY_METERS_PER_SECOND;
  private double heightAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION_METERS_PER_SECOND_SQUARED;
  private double heightJerk = ElevatorConstants.MOTION_MAGIC_JERK_METERS_PER_SECOND_CUBED;
  private double profileHeightAcceleration, profilePivotAcceleration;
  private TrapezoidProfile pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      ElevatorConstants.PIVOT_PROFILE_VELOCITY_RADIANS_PER_SECOND,
      ElevatorConstants.PIVOT_PROFILE_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
  private TrapezoidProfile.State heightSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();
  // Sim starts at the known home height, so the height reference is valid from construction;
  // the manual homing flow still runs if requested.
  private boolean heightReferenceValid = true;
  private boolean configurationValid = true;

  public ElevatorIOSim() {
    heightSim = new ElevatorSim(ElevatorConstants.SIM_MOTOR, ElevatorConstants.REDUCTION,
        ElevatorConstants.SIM_CARRIAGE_MASS.in(Kilograms),
        ElevatorConstants.metersPerAverageRotation() / (2 * Math.PI),
        ElevatorConstants.MIN_POSITION.in(Meters), ElevatorConstants.MAX_POSITION.in(Meters),
        ElevatorConstants.SIMULATE_GRAVITY, ElevatorConstants.HOME_POSITION.in(Meters));
    pivotSim = new SingleJointedArmSim(ElevatorConstants.SIM_MOTOR,
        ElevatorConstants.SIM_PIVOT_REDUCTION, ElevatorConstants.SIM_PIVOT_MOI_KG_METERS_SQUARED,
        ElevatorConstants.SIM_PIVOT_LENGTH.in(Meters),
        ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians), ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians),
        ElevatorConstants.SIMULATE_GRAVITY, ElevatorConstants.SIM_INITIAL_PIVOT_ANGLE.in(Radians));
    setPIDGains(Axis.HEIGHT, heightPositionGains);
    setPIDGains(Axis.PIVOT, pivotPositionGains);
    // Deliberately requires the same manual calibration as real mode.
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double dt = Constants.loopPeriodSecs;
    if ((!heightReferenceValid && controlState != ControlState.HOMING) || !configurationValid || RobotState.isDisabled()
      || (controlState == ControlState.HOMING && Math.abs(pivotSim.getAngle() - pivotGoal)
            > ElevatorConstants.HOMING_MAX_PIVOT_ERROR.in(Radians))) {
      stop();
    }
    double heightTurnsPerSecond = heightSim.getVelocity()
        / ElevatorConstants.metersPerAverageRotation();
    double pivotTurnsPerSecond = pivotSim.getVelocity() / (2 * Math.PI);
    double leaderRotorSpeed = 2 * Math.PI * (heightTurnsPerSecond * ElevatorConstants.REDUCTION
        + pivotTurnsPerSecond * ElevatorConstants.SIM_PIVOT_REDUCTION);
    double followerRotorSpeed = 2 * Math.PI * (heightTurnsPerSecond * ElevatorConstants.REDUCTION
        - pivotTurnsPerSecond * ElevatorConstants.SIM_PIVOT_REDUCTION);

    if (controlState == ControlState.POSITION) {
      advanceHeightProfile(dt);
      double previous = pivotSetpoint.velocity;
      pivotSetpoint = pivotProfile.calculate(dt, pivotSetpoint,
          new TrapezoidProfile.State(pivotGoal, 0));
      profilePivotAcceleration = (pivotSetpoint.velocity - previous) / dt;
    }
    if (controlState == ControlState.HOMING) {
      double differenceVolts = homingPivotPID.calculate(
          pivotSim.getAngle() / (2 * Math.PI), pivotGoal / (2 * Math.PI))
          + ElevatorConstants.HOMING_PIVOT_VOLTAGE_GAINS.kG() * Math.cos(pivotSim.getAngle());
      leaderVolts = homingVolts + differenceVolts;
      followerVolts = homingVolts - differenceVolts;
    } else if (controlState == ControlState.STOPPED || controlState == ControlState.MUSIC) {
      leaderVolts = followerVolts = 0;
    } else {
      double averageAmps = heightCurrent();
      double differenceAmps = pivotCurrent();
      leaderVolts = singleMotor.getVoltage(singleMotor.getTorque(clampCurrent(averageAmps + differenceAmps)),
          leaderRotorSpeed);
      followerVolts = singleMotor.getVoltage(singleMotor.getTorque(clampCurrent(averageAmps - differenceAmps)),
          followerRotorSpeed);
    }
    leaderVolts = Math.clamp(leaderVolts, -12, 12);
    followerVolts = Math.clamp(followerVolts, -12, 12);
    double averageVolts = (leaderVolts + followerVolts) / 2;
    double differenceVolts = (leaderVolts - followerVolts) / 2;
    // Enforce physical-axis limits after mixing, so suppressing one axis cannot create the other.
    if ((controlState != ControlState.HOMING && averageVolts < 0
            && heightSim.getPosition() <= ElevatorConstants.MIN_POSITION.in(Meters))
          || (averageVolts > 0 && heightSim.getPosition() >= ElevatorConstants.MAX_POSITION.in(Meters))) {
      averageVolts = 0;
    }
    if ((differenceVolts < 0 && pivotSim.getAngle() <= ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians))
      || (differenceVolts > 0 && pivotSim.getAngle() >= ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians))) {
      differenceVolts = 0;
    }
    leaderVolts = averageVolts + differenceVolts;
    followerVolts = averageVolts - differenceVolts;
    leaderAmps = singleMotor.getCurrent(leaderRotorSpeed, leaderVolts);
    followerAmps = singleMotor.getCurrent(followerRotorSpeed, followerVolts);
    heightSim.setInputVoltage(averageVolts);
    pivotSim.setInputVoltage(differenceVolts);
    heightSim.update(dt);
    pivotSim.update(dt);

    inputs.hardwareConfigurationValid = configurationValid;
    inputs.heightReferenceValid = heightReferenceValid;
    inputs.homingActive = controlState == ControlState.HOMING;
    inputs.height = Meters.of(heightSim.getPosition());
    inputs.heightVelocity = MetersPerSecond.of(heightSim.getVelocity());
    inputs.pivotAngle = Radians.of(pivotSim.getAngle());
    inputs.pivotVelocity = RadiansPerSecond.of(pivotSim.getVelocity());
    inputs.leaderAppliedVoltage = Volts.of(leaderVolts);
    inputs.followerAppliedVoltage = Volts.of(followerVolts);
    // Electrical samples share the pre-step working point used to compute these motor currents.
    inputs.leaderRotorVelocityRadPerSec = leaderRotorSpeed;
    inputs.followerRotorVelocityRadPerSec = followerRotorSpeed;
    inputs.leaderStatorCurrent = Amps.of(leaderAmps * leaderRotorSpeed < 0
        ? -Math.abs(leaderAmps) : Math.abs(leaderAmps));
    inputs.followerStatorCurrent = Amps.of(followerAmps * followerRotorSpeed < 0
        ? -Math.abs(followerAmps) : Math.abs(followerAmps));
    inputs.leaderStatorCurrentAvailable = inputs.followerStatorCurrentAvailable = true;
    inputs.leaderSupplyCurrentAvailable = inputs.followerSupplyCurrentAvailable = true;
    inputs.leaderSupplyCurrent = Amps.of(leaderAmps * leaderVolts / RobotController.getBatteryVoltage());
    inputs.followerSupplyCurrent = Amps.of(followerAmps * followerVolts / RobotController.getBatteryVoltage());
    inputs.leaderTemperature = inputs.followerTemperature = Celsius.of(35);
  }

  private void advanceHeightProfile(double dt) {
    TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(heightCruise, heightAcceleration));
    TrapezoidProfile.State requested = profile.calculate(dt, heightSetpoint,
        new TrapezoidProfile.State(heightGoal, 0));
    if (heightJerk == 0) {
      profileHeightAcceleration = (requested.velocity - heightSetpoint.velocity) / dt;
      heightSetpoint = requested;
      return;
    }
    double acceleration = Math.clamp((requested.velocity - heightSetpoint.velocity) / dt,
        profileHeightAcceleration - heightJerk * dt, profileHeightAcceleration + heightJerk * dt);
    double velocity = Math.clamp(heightSetpoint.velocity + acceleration * dt, -heightCruise, heightCruise);
    double position = heightSetpoint.position + (heightSetpoint.velocity + velocity) * dt / 2;
    if ((heightGoal - heightSetpoint.position) * (heightGoal - position) <= 0) {
      position = heightGoal;
      velocity = acceleration = 0;
    }
    heightSetpoint = new TrapezoidProfile.State(position, velocity);
    profileHeightAcceleration = acceleration;
  }
  private double heightCurrent() {
    double scale = ElevatorConstants.metersPerAverageRotation();
    return heightPositionPID.calculate(heightSim.getPosition() / scale, heightSetpoint.position / scale)
        + feedforward(heightPositionGains, heightSetpoint.velocity / scale,
            profileHeightAcceleration / scale, heightPositionGains.kG());
  }
  private double pivotCurrent() {
    return pivotPositionPID.calculate(pivotSim.getAngle() / (2 * Math.PI),
            pivotSetpoint.position / (2 * Math.PI))
        + feedforward(pivotPositionGains, pivotSetpoint.velocity / (2 * Math.PI),
            profilePivotAcceleration / (2 * Math.PI),
            pivotPositionGains.kG() * Math.cos(pivotSim.getAngle()));
  }
  private static double feedforward(ClosedLoopGains g, double velocity, double acceleration, double gravity) {
    double staticAmps = Math.abs(velocity) > 1e-6 ? Math.copySign(g.kS(), velocity) : 0;
    return staticAmps + g.kV() * velocity + g.kA() * acceleration + gravity;
  }
  private static double clampCurrent(double current) {
    return Math.clamp(current, ElevatorConstants.PEAK_REVERSE_TORQUE_CURRENT_AMPS,
        ElevatorConstants.PEAK_FORWARD_TORQUE_CURRENT_AMPS);
  }

  @Override
  public void setPosition(Distance height, Angle pivot) {
    if (controlState == ControlState.HOMING) { stop(); return; }
    if (controlState != ControlState.POSITION) {
      heightPositionPID.reset();
      pivotPositionPID.reset();
      heightSetpoint = new TrapezoidProfile.State(
          heightSim.getPosition(), heightSim.getVelocity());
      pivotSetpoint = new TrapezoidProfile.State(
          pivotSim.getAngle(), pivotSim.getVelocity());
      profileHeightAcceleration = profilePivotAcceleration = 0;
    }
    controlState = ControlState.POSITION;
    heightGoal = height.in(Meters);
    pivotGoal = pivot.in(Radians);
  }

  @Override
  public boolean resetHeight(Distance height) {
    if (!RobotState.isDisabled() || !configurationValid
        || !Double.isFinite(height.in(Meters))
        || height.in(Meters) < ElevatorConstants.MIN_POSITION.in(Meters)
        || height.in(Meters) > ElevatorConstants.MAX_POSITION.in(Meters)
        || Math.abs(heightSim.getVelocity()) > ElevatorConstants.RESET_MAX_HEIGHT_VELOCITY.in(MetersPerSecond)
        || Math.abs(pivotSim.getVelocity()) > ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond)) {
      return false;
    }
    stop();
    heightSim.setState(height.in(Meters), 0);
    heightReferenceValid = true;
    return true;
  }

  @Override
  public boolean beginHoming(Angle pivotAngle) {
    stop();
    if (!RobotState.isEnabled() || !configurationValid) return false;
    ClosedLoopGains gains = ElevatorConstants.HOMING_PIVOT_VOLTAGE_GAINS;
    homingPivotPID.setPID(gains.kP(), gains.kI(), gains.kD());
    homingPivotPID.reset();
    heightReferenceValid = false;
    pivotGoal = pivotAngle.in(Radians);
    controlState = ControlState.HOMING;
    return true;
  }

  @Override
  public boolean setHomingVoltage(Voltage voltage) {
    if (controlState != ControlState.HOMING || !RobotState.isEnabled() || !configurationValid
        || !Double.isFinite(voltage.in(Volts)) || voltage.in(Volts) >= 0
        || Math.abs(pivotSim.getAngle() - pivotGoal)
            > ElevatorConstants.HOMING_MAX_PIVOT_ERROR.in(Radians)) {
      stop();
      return false;
    }
    homingVolts = Math.clamp(voltage.in(Volts), ElevatorConstants.HOMING_VOLTAGE.in(Volts), 0);
    return true;
  }

  @Override
  public boolean completeHoming() {
    if (controlState != ControlState.HOMING || !RobotState.isEnabled() || !configurationValid
      || Math.abs(heightSim.getVelocity())
            > ElevatorConstants.HOMING_VELOCITY_THRESHOLD.in(MetersPerSecond)
        || Math.abs(pivotSim.getVelocity())
            > ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond)
        || Math.abs(pivotSim.getAngle() - pivotGoal)
            > ElevatorConstants.HOMING_MAX_PIVOT_ERROR.in(Radians)
        || Math.abs(leaderAmps) < ElevatorConstants.HOMING_CURRENT_THRESHOLD.in(Amps)
        || Math.abs(followerAmps) < ElevatorConstants.HOMING_CURRENT_THRESHOLD.in(Amps)) {
      stop();
      return false;
    }
    heightSim.setState(ElevatorConstants.HOME_POSITION.in(Meters), 0);
    heightReferenceValid = true;
    stop();
    return true;
  }
  @Override
  public void setPIDGains(Axis axis, ClosedLoopGains gains) {
    if (!gains.isFinite() || gains.kP() <= 0) {
      configurationValid = false;
      stop(); return;
    }
    if (axis == Axis.HEIGHT) {
      heightPositionGains = gains;
      heightPositionPID.setPID(gains.kP(), gains.kI(), gains.kD());
    } else {
      pivotPositionGains = gains;
      pivotPositionPID.setPID(gains.kP(), gains.kI(), gains.kD());
    }
  }
  @Override
  public void setMotionMagicConfig(double cruise, double acceleration, double jerk) {
    if (!Double.isFinite(cruise) || cruise <= 0 || !Double.isFinite(acceleration)
        || acceleration <= 0 || !Double.isFinite(jerk) || jerk < 0) {
      configurationValid = false; stop(); return;
    }
    heightCruise = cruise;
    heightAcceleration = acceleration;
    heightJerk = jerk;
  }
  @Override
  public void setPivotProfileConfig(double velocity, double acceleration) {
    if (!Double.isFinite(velocity) || velocity <= 0 || !Double.isFinite(acceleration) || acceleration <= 0) {
      configurationValid = false; stop(); return;
    }
    pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(velocity, acceleration));
  }
  @Override
  public boolean setMusicTone(Frequency frequency) { stop(); return false; }
  @Override
  public void stopMusicTone() { stop(); }
  @Override
  public void stop() {
    controlState = ControlState.STOPPED;
    homingVolts = leaderVolts = followerVolts = 0;
  }
}
