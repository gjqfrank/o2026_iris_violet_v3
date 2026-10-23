// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.*;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.units.measure.*;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.system.Timer;

import frc.robot.commands.ElevatorHomingCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.energy.BatteryChannel;
import frc.robot.energy.BatteryLogger;
import frc.robot.subsystems.elevator.ElevatorIO.Axis;
import frc.robot.subsystems.elevator.ElevatorIO.ClosedLoopGains;
import frc.robot.subsystems.elevator.ElevatorIO.ControlState;
import frc.robot.subsystems.elevator.ElevatorConstants.ArmPose;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Owns both motors and both axes; commands must coordinate height and pivot under one requirement. */
public class Elevator implements Mechanism {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final int tuningId = System.identityHashCode(this);
  private final Alert mechanismUnavailable =
      new Alert("hardware unconfigured","Elevator differential configuration/mechanism unavailable.", Level.HIGH);
  private final Alert heightNotCalibrated =
      new Alert("not homed","Elevator height requires homing or manual home reset.", Level.MEDIUM);

  private ControlState controlState = ControlState.STOPPED;
  private Distance heightTarget = ElevatorConstants.HOME_POSITION;
  private Angle pivotTarget = Radians.zero();
  private Voltage homingVoltage = Volts.zero();
  private boolean brakeModeEnabled = ElevatorConstants.BRAKE_MODE_ENABLED;
  private double heightSettledSince = Double.NaN;
  private double pivotSettledSince = Double.NaN;
  private double arrivalTimestamp = Double.NaN;

  public Elevator(ElevatorIO io) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    this.io = io;
  }

  private void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    mechanismUnavailable.set(!configurationReady() || !inputs.mechanismFault.isEmpty());
    heightNotCalibrated.set(!isHomed());

    if (!configurationReady() || (!isHomed() && controlState != ControlState.HOMING)
        || (controlState == ControlState.HOMING && !isHomingHealthy())
        || !inputs.mechanismFault.isEmpty()
        || (RobotState.isDisabled() && controlState != ControlState.MUSIC)) {
      stop();
    }

    updateArrivalState();
    Logger.recordOutput("Elevator/AtHeight", atHeight());
    Logger.recordOutput("Elevator/AtPivotAngle", atPivotAngle());
    Logger.recordOutput("Elevator/AtTargets", atTargets());
    Logger.recordOutput("Elevator/ControlState", controlState.name());
    Logger.recordOutput("Elevator/Homed", isHomed());
    Logger.recordOutput("Elevator/HomingActive", controlState == ControlState.HOMING);
    Logger.recordOutput("Elevator/HeightTargetMeters", heightTarget.in(Meters));
    Logger.recordOutput("Elevator/PivotTargetDegrees", pivotTarget.in(Degrees));
    Logger.recordOutput("Elevator/HomingVoltage", homingVoltage.in(Volts));
    Logger.recordOutput("Elevator/BrakeModeEnabled", brakeModeEnabled);
    updateTunableGains();
    // Differential motors have different working points, so each is reported on its own channel.
    BatteryLogger.getInstance()
        .reportSupplyCurrent(
            BatteryChannel.ELEVATOR_LEADER,
            inputs.leaderSupplyCurrentAvailable ? inputs.leaderSupplyCurrent.in(Amps) : 0.0);
    BatteryLogger.getInstance()
        .reportSupplyCurrent(
            BatteryChannel.ELEVATOR_FOLLOWER,
            inputs.followerSupplyCurrentAvailable ? inputs.followerSupplyCurrent.in(Amps) : 0.0);
  }

  /** Retains the pivot target during position control; otherwise captures its measured angle. */
  public void setHeight(Distance height) {
    setTargets(height, controlState == ControlState.POSITION ? pivotTarget : inputs.pivotAngle);
  }

  /** Retains the height target during position control; otherwise captures measured height. */
  public void setPivotAngle(Angle angle) {
    setTargets(controlState == ControlState.POSITION ? heightTarget : inputs.height, angle);
  }

  /** Direct two-axis setpoint; coordinated safe travel belongs to ElevatorMoveSafelyCommand. */
  public void setTargets(Distance height, Angle angle) {
    commandTargets(height, angle);
  }

  public void setTargets(ArmPose pose) {
    setTargets(pose.height(), pose.pivot());
  }

  /** True once either direct or safe travel has settled at the commanded pose. */
  public boolean hasReached(ArmPose pose) {
    return isCommandedTo(pose) && atTargets();
  }

  /** True while the position loop is commanded to exactly this pose, arrived or not. */
  public boolean isCommandedTo(ArmPose pose) {
    return controlState == ControlState.POSITION
        && Double.compare(heightTarget.in(Meters), pose.height().in(Meters)) == 0
        && Double.compare(pivotTarget.in(Radians), pose.pivot().in(Radians)) == 0;
  }

  /** Holds the arm exactly where it physically is right now. */
  public void holdStill() {
    setTargets(inputs.height, inputs.pivotAngle);
  }

  private void commandTargets(Distance height, Angle angle) {
    if (!canMove() || !Double.isFinite(height.in(Meters))
        || !Double.isFinite(angle.in(Radians))) { stop(); return; }
    Distance newHeightTarget = clampHeight(height);
    Angle newPivotTarget = clampPivot(angle);
    if (controlState != ControlState.POSITION
        || Double.compare(newHeightTarget.in(Meters), heightTarget.in(Meters)) != 0) {
      heightSettledSince = Double.NaN;
    }
    if (controlState != ControlState.POSITION
        || Double.compare(newPivotTarget.in(Radians), pivotTarget.in(Radians)) != 0) {
      pivotSettledSince = Double.NaN;
    }
    heightTarget = newHeightTarget;
    pivotTarget = newPivotTarget;
    controlState = ControlState.POSITION;
    io.setPosition(heightTarget, pivotTarget);
  }

  /**
   * Operator must physically place the elevator at the supplied height first.
   * Only allowed while disabled and stationary; pivot calibration is not changed.
   */
  public boolean resetHeight(Distance height) {
    boolean stationary = Math.abs(inputs.heightVelocity.in(MetersPerSecond))
            <= ElevatorConstants.RESET_MAX_HEIGHT_VELOCITY.in(MetersPerSecond)
        && Math.abs(inputs.pivotVelocity.in(RadiansPerSecond))
            <= ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond);
    if (RobotState.isEnabled() || !stationary || !configurationReady()
        || !Double.isFinite(height.in(Meters))
        || height.in(Meters) < ElevatorConstants.MIN_POSITION.in(Meters)
        || height.in(Meters) > ElevatorConstants.MAX_POSITION.in(Meters)) {
      return false;
    }
    stop();
    boolean succeeded = io.resetHeight(height);
    inputs.heightReferenceValid = succeeded;
    if (succeeded) {
      inputs.height = height;
      heightTarget = height;
      pivotTarget = inputs.pivotAngle;
    }
    Logger.recordOutput("Elevator/ManualHeightResetSucceeded", succeeded);
    return succeeded;
  }

  /** Captures pivot for holding while height moves without a calibrated reference. */
  public boolean beginHoming() {
    stop();
    if (RobotState.isDisabled() || !configurationReady() || !ElevatorConstants.homingConfigured()
        || inputs.pivotAngle.in(Radians) < ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians)
        || inputs.pivotAngle.in(Radians) > ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians)
        || Math.abs(inputs.pivotVelocity.in(RadiansPerSecond))
            > ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond)) {
      return false;
    }
    pivotTarget = inputs.pivotAngle;
    if (!io.beginHoming(pivotTarget)) return false;
    inputs.heightReferenceValid = false;
    inputs.homingActive = true;
    inputs.mechanismFault = "";
    controlState = ControlState.HOMING;
    return true;
  }

  public boolean setHomingVoltage(Voltage voltage) {
    if (!isHomingHealthy() || RobotState.isDisabled()
        || !Double.isFinite(voltage.in(Volts)) || voltage.in(Volts) >= 0) {
      stop();
      return false;
    }
    homingVoltage = Volts.of(Math.clamp(voltage.in(Volts),
        ElevatorConstants.HOMING_VOLTAGE.in(Volts), 0));
    if (!io.setHomingVoltage(homingVoltage)) {
      stop();
      return false;
    }
    return true;
  }

  public boolean isHomingHealthy() {
    return controlState == ControlState.HOMING && inputs.homingActive && configurationReady()
        && inputs.mechanismFault.isEmpty()
        && Math.abs(pivotTarget.in(Radians) - inputs.pivotAngle.in(Radians))
            <= ElevatorConstants.HOMING_MAX_PIVOT_ERROR.in(Radians);
  }

  public boolean completeHoming() {
    boolean succeeded = isHomingHealthy() && io.completeHoming();
    stop();
    inputs.heightReferenceValid = succeeded;
    if (succeeded) {
      inputs.height = ElevatorConstants.HOME_POSITION;
      heightTarget = inputs.height;
    }
    return succeeded;
  }

  public Current getLeaderStatorCurrent() { return inputs.leaderStatorCurrent; }
  public Current getFollowerStatorCurrent() { return inputs.followerStatorCurrent; }

  public boolean setMusicTone(Frequency frequency) {
    if (!configurationReady() || !isHomed() || !RobotState.isDisabled()) {
      stop(); return false;
    }
    stop();
    controlState = ControlState.MUSIC;
    return io.setMusicTone(frequency);
  }
  public void stopMusicTone() { stop(); }
  public void setBrakeModeEnabled(boolean enabled) {
    brakeModeEnabled = enabled;
    io.setBrakeModeEnabled(enabled);
  }

  @AutoLogOutput(key = "Elevator/MeasuredHeight")
  public Distance getHeight() { return inputs.height; }
  public LinearVelocity getHeightVelocity() { return inputs.heightVelocity; }
  @AutoLogOutput(key = "Elevator/MeasuredPivotAngle")
  public Angle getPivotAngle() { return inputs.pivotAngle; }
  public AngularVelocity getPivotVelocity() { return inputs.pivotVelocity; }
  public boolean isHomed() { return inputs.heightReferenceValid; }
  /** Checks configuration, height reference and mechanism fault. */
  public boolean isPositionControlReady() { return canMove(); }
  /** Position and velocity must remain within tolerance for the configured settling time. */
  public boolean atHeight() {
    return heightWithinArrivalTolerance() && hasSettled(heightSettledSince);
  }
  public boolean atPivotAngle() {
    return pivotWithinArrivalTolerance() && hasSettled(pivotSettledSince);
  }
  /** Both axes must have satisfied their arrival conditions over the same settling interval. */
  public boolean atTargets() { return atHeight() && atPivotAngle(); }

  private boolean heightWithinArrivalTolerance() {
    return RobotState.isEnabled() && canMove() && controlState == ControlState.POSITION
        && Math.abs(heightTarget.in(Meters) - inputs.height.in(Meters))
            <= ElevatorConstants.POSITION_TOLERANCE.in(Meters)
        && Math.abs(inputs.heightVelocity.in(MetersPerSecond))
            <= ElevatorConstants.HEIGHT_ARRIVAL_VELOCITY_TOLERANCE.in(MetersPerSecond);
  }

  private boolean pivotWithinArrivalTolerance() {
    return RobotState.isEnabled() && canMove() && controlState == ControlState.POSITION
        && Math.abs(pivotTarget.in(Radians) - inputs.pivotAngle.in(Radians))
            <= ElevatorConstants.PIVOT_TOLERANCE.in(Radians)
        && Math.abs(inputs.pivotVelocity.in(RadiansPerSecond))
            <= ElevatorConstants.PIVOT_ARRIVAL_VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  private boolean hasSettled(double settledSince) {
    return Double.isFinite(settledSince)
        && arrivalTimestamp - settledSince >= ElevatorConstants.ARRIVAL_SETTLE_TIME.in(Seconds);
  }

  private void updateArrivalState() {
    double now = Timer.getTimestamp();
    arrivalTimestamp = now;
    // Evaluate during periodic IO updates; queries never advance the settling clock.
    if (!heightWithinArrivalTolerance()) heightSettledSince = Double.NaN;
    else if (!Double.isFinite(heightSettledSince)) heightSettledSince = now;
    if (!pivotWithinArrivalTolerance()) pivotSettledSince = Double.NaN;
    else if (!Double.isFinite(pivotSettledSince)) pivotSettledSince = now;
  }

  private void resetArrivalState() {
    heightSettledSince = Double.NaN;
    pivotSettledSince = Double.NaN;
  }

  public void stop() {
    resetArrivalState();
    inputs.homingActive = false;
    controlState = ControlState.STOPPED;
    homingVoltage = Volts.zero();
    heightTarget = inputs.height;
    pivotTarget = inputs.pivotAngle;
    io.stop();
  }

  /** Finite direct motion, for a path whose swept space is already known to be clear. */
  public Command moveToCommand(ArmPose pose) {
    return run(coroutine -> {
          if (!pose.configured() || !isPositionControlReady()) {
            stop();
            coroutine.park();
          }
          setTargets(pose);
          coroutine.waitUntil(() -> hasReached(pose) || !isPositionControlReady());
          if (!isPositionControlReady()) {
            stop();
            coroutine.park();
          }
        })
        .whenCanceled(this::stop)
        .named("Elevator:MoveToPose");
  }

  public Command runPositionCommand(Supplier<Distance> height, Supplier<Angle> pivot) {
    BooleanSupplier isFinished = () -> false;
    return run(
            c -> {
              while (!isFinished.getAsBoolean()) {
                setTargets(height.get(), pivot.get());
                c.yield();
              }
            })
        .whenCanceled(this::stop)
        .named("Elevator:Position");
  }
  public Command runHeightCommand(Supplier<Distance> height) {
    BooleanSupplier isFinished = () -> false;
    return run(
            c -> {
              while (!isFinished.getAsBoolean()) {
                setHeight(height.get());
                c.yield();
              }
            })
        .whenCanceled(this::stop)
        .named("Elevator:Height");
  }
  public Command runPivotCommand(Supplier<Angle> angle) {
    BooleanSupplier isFinished = () -> false;
    return run(
            c -> {
              while (!isFinished.getAsBoolean()) {
                setPivotAngle(angle.get());
                c.yield();
              }
            })
        .whenCanceled(this::stop)
        .named("Elevator:Pivot");
  }

  public Command homingCommand(){
    return new ElevatorHomingCommand(this);
  }

  private boolean configurationReady() {
    return ElevatorConstants.mechanismConfigured() && inputs.hardwareConfigurationValid;
  }
  private boolean canMove() {
    return configurationReady() && isHomed() && inputs.mechanismFault.isEmpty();
  }
  private static Distance clampHeight(Distance height) {
    return Meters.of(Math.clamp(height.in(Meters),
        ElevatorConstants.MIN_POSITION.in(Meters), ElevatorConstants.MAX_POSITION.in(Meters)));
  }
  private static Angle clampPivot(Angle angle) {
    return Radians.of(Math.clamp(angle.in(Radians),
        ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians), ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians)));
  }

  private void updateTunableGains() {
    if (Constants.getMode() == Mode.REPLAY) return;
    if (Constants.getMode() == Mode.SIM) {
      updatePositionGains(Axis.HEIGHT,
          ElevatorTuning.SIM_POSITION_KP, ElevatorTuning.SIM_POSITION_KI, ElevatorTuning.SIM_POSITION_KD,
          ElevatorTuning.SIM_POSITION_KS, ElevatorTuning.SIM_POSITION_KV, ElevatorTuning.SIM_POSITION_KA,
          ElevatorTuning.SIM_POSITION_KG);
      updatePositionGains(Axis.PIVOT,
          ElevatorTuning.SIM_PIVOT_POSITION_KP, ElevatorTuning.SIM_PIVOT_POSITION_KI, ElevatorTuning.SIM_PIVOT_POSITION_KD,
          ElevatorTuning.SIM_PIVOT_POSITION_KS, ElevatorTuning.SIM_PIVOT_POSITION_KV, ElevatorTuning.SIM_PIVOT_POSITION_KA,
          ElevatorTuning.SIM_PIVOT_POSITION_KG);
    } else {
      updatePositionGains(Axis.HEIGHT,
          ElevatorTuning.POSITION_KP, ElevatorTuning.POSITION_KI, ElevatorTuning.POSITION_KD,
          ElevatorTuning.POSITION_KS, ElevatorTuning.POSITION_KV, ElevatorTuning.POSITION_KA,
          ElevatorTuning.POSITION_KG);
      updatePositionGains(Axis.PIVOT,
          ElevatorTuning.PIVOT_POSITION_KP, ElevatorTuning.PIVOT_POSITION_KI, ElevatorTuning.PIVOT_POSITION_KD,
          ElevatorTuning.PIVOT_POSITION_KS, ElevatorTuning.PIVOT_POSITION_KV, ElevatorTuning.PIVOT_POSITION_KA,
          ElevatorTuning.PIVOT_POSITION_KG);
    }
    LoggedTunableNumber.ifChanged(tuningId,
        () -> io.setMotionMagicConfig(ElevatorTuning.MOTION_MAGIC_CRUISE_VELOCITY.get(),
            ElevatorTuning.MOTION_MAGIC_ACCELERATION.get(), ElevatorTuning.MOTION_MAGIC_JERK.get()),
        ElevatorTuning.MOTION_MAGIC_CRUISE_VELOCITY,
        ElevatorTuning.MOTION_MAGIC_ACCELERATION, ElevatorTuning.MOTION_MAGIC_JERK);
    LoggedTunableNumber.ifChanged(tuningId,
        () -> io.setPivotProfileConfig(ElevatorTuning.PIVOT_VELOCITY_LIMIT.get(),
            ElevatorTuning.PIVOT_ACCELERATION.get()),
        ElevatorTuning.PIVOT_VELOCITY_LIMIT, ElevatorTuning.PIVOT_ACCELERATION);
  }

  private void updatePositionGains(Axis axis, LoggedTunableNumber... gains) {
    LoggedTunableNumber.ifChanged(tuningId,
        values -> io.setPIDGains(axis, new ClosedLoopGains(
            values[0], values[1], values[2], values[3], values[4], values[5], values[6])),
        gains);
  }
}
