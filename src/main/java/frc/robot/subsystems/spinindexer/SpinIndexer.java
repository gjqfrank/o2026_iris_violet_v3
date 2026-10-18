// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.spinindexer;

import static org.wpilib.units.Units.*;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Voltage;
import org.wpilib.system.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralHandlingConstants;
import frc.robot.energy.BatteryChannel;
import frc.robot.energy.BatteryLogger;
import frc.robot.subsystems.common.ControlMode;
import frc.robot.util.LoggedTunableNumber;
import java.util.Objects;
import java.util.OptionalInt;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SpinIndexer implements Mechanism {
  private final SpinIndexerIO io;
  private final SpinIndexerIOInputsAutoLogged inputs = new SpinIndexerIOInputsAutoLogged();
  private final Supplier<Distance> armHeight;


  private final int tuningId = System.identityHashCode(this);

  private Voltage commandedVoltage = Volts.of(0.0);
  private double commandedCurrentAmps = 0.0;
  private AngularVelocity commandedVelocity = RotationsPerSecond.of(0.0);
  private Angle commandedPosition = Rotations.of(0.0);
  private ControlMode controlMode = ControlMode.STOPPED;
  private final SysIdRoutine sysId;
  private double slotSettledSince = Double.NaN;
  private double angleSettledSince = Double.NaN;
  public SpinIndexer(SpinIndexerIO io, Supplier<Distance> armHeight) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    this.io = io;
    this.armHeight = Objects.requireNonNull(armHeight, "armHeight");
    // SysId's voltage-typed waveform is interpreted numerically as amps, matching TorqueCurrentFOC.
    Logger.recordOutput("SpinIndexer/SysId/InputUnit", "A");
    sysId =
      new SysIdRoutine(
        new SysIdRoutine.Config(
          Volts.per(Second).of(SpinIndexerConstants.SYSID_RAMP_RATE.in(Amps.per(Second))),
          Volts.of(SpinIndexerConstants.SYSID_STEP_CURRENT.in(Amps)),
          SpinIndexerConstants.SYSID_QUASISTATIC_DURATION,
          state -> Logger.recordOutput("SpinIndexer/SysId/State", state.toString())),
        new SysIdRoutine.SysIdMechanism(
          voltage -> runCurrentCharacterization(voltage.in(Volts)), null, this));
  }

  private void periodic() {
    double t0 = Timer.getTimestamp();
    io.updateInputs(inputs);
    Logger.processInputs("SpinIndexer", inputs);

    if (!inputs.hardwareConfigurationValid) {
      stop();
    }

    Logger.recordOutput("SpinIndexer/CommandedVoltage", commandedVoltage.in(Volts), Volts);
    Logger.recordOutput("SpinIndexer/SysId/CommandedCurrentAmps",
        controlMode == ControlMode.TORQUE_CURRENT ? commandedCurrentAmps : 0.0, Amps);
    Logger.recordOutput(
        "SpinIndexer/CommandedVelocity",
        commandedVelocity.in(RotationsPerSecond),
        RotationsPerSecond);
    Logger.recordOutput(
        "SpinIndexer/CommandedPosition", commandedPosition.in(Rotations), Rotations);
    Logger.recordOutput("SpinIndexer/PositionError", getPositionError().in(Rotations), Rotations);
    Logger.recordOutput("SpinIndexer/ControlMode", controlMode.getValue());
    Logger.recordOutput("SpinIndexer/Inventory/AlignedSlot", getAlignedSlot().orElse(-1));
    if(Constants.getMode() == Constants.Mode.REAL){
      LoggedTunableNumber.ifChanged(
        tuningId,
        () ->
            io.setVelocityPIDGains(
                SpinIndexerTuning.VELOCITY_KP.get(),
                SpinIndexerTuning.VELOCITY_KI.get(),
                SpinIndexerTuning.VELOCITY_KD.get(),
                SpinIndexerTuning.VELOCITY_KS.get(),
                SpinIndexerTuning.VELOCITY_KV.get(),
                SpinIndexerTuning.VELOCITY_KA.get()),
        SpinIndexerTuning.VELOCITY_KP,
        SpinIndexerTuning.VELOCITY_KI,
        SpinIndexerTuning.VELOCITY_KD,
        SpinIndexerTuning.VELOCITY_KS,
        SpinIndexerTuning.VELOCITY_KV,
        SpinIndexerTuning.VELOCITY_KA);

      LoggedTunableNumber.ifChanged(
        tuningId,
        () ->
            io.setPositionPIDGains(
                SpinIndexerTuning.POSITION_KP.get(),
                SpinIndexerTuning.POSITION_KI.get(),
                SpinIndexerTuning.POSITION_KD.get(),
                SpinIndexerTuning.POSITION_KS.get(),
                SpinIndexerTuning.POSITION_KV.get(),
                SpinIndexerTuning.POSITION_KA.get()),
        SpinIndexerTuning.POSITION_KP,
        SpinIndexerTuning.POSITION_KI,
        SpinIndexerTuning.POSITION_KD,
        SpinIndexerTuning.POSITION_KS,
        SpinIndexerTuning.POSITION_KV,
        SpinIndexerTuning.POSITION_KA);

      LoggedTunableNumber.ifChanged(
        tuningId,
        () ->
            io.setMotionMagicConfig(
                SpinIndexerTuning.MOTION_MAGIC_CRUISE_VELOCITY.get(),
                SpinIndexerTuning.MOTION_MAGIC_ACCELERATION.get(),
                SpinIndexerTuning.MOTION_MAGIC_JERK.get()),
        SpinIndexerTuning.MOTION_MAGIC_CRUISE_VELOCITY,
        SpinIndexerTuning.MOTION_MAGIC_ACCELERATION,
        SpinIndexerTuning.MOTION_MAGIC_JERK);
    }

    Logger.recordOutput("Timing/SpinIndexerMs", (Timer.getTimestamp() - t0) * 1000.0);
    BatteryLogger.getInstance()
        .reportSupplyCurrent(
            BatteryChannel.SPININDEXER_MOTOR,
            inputs.supplyCurrentAvailable ? inputs.supplyCurrent.in(Amps) : 0.0);
  }

  public void setVoltage(Voltage voltage) {
    Objects.requireNonNull(voltage, "voltage");
    if (!Double.isFinite(voltage.in(Volts)) || !inputs.hardwareConfigurationValid) {
      stop();
      return;
    }
    controlMode = ControlMode.VOLTAGE;
    commandedVoltage = Volts.of(Math.clamp(voltage.in(Volts), -12.0, 12.0));
    commandedVelocity = RotationsPerSecond.of(0.0);
    commandedPosition = inputs.position;
    io.setVoltage(commandedVoltage);
  }

  public void setVelocity(AngularVelocity velocity) {
    Objects.requireNonNull(velocity, "velocity");
    if (!Double.isFinite(velocity.in(RotationsPerSecond)) || !inputs.hardwareConfigurationValid) {
      stop();
      return;
    }
    controlMode = ControlMode.VELOCITY;
    commandedVoltage = Volts.of(0.0);
    commandedVelocity = velocity;
    commandedPosition = inputs.position;
    io.setVelocity(velocity);
  }

  public void setPosition(Angle position) {
    Objects.requireNonNull(position, "position");
    if (!Double.isFinite(position.in(Rotations)) || !inputs.hardwareConfigurationValid) {
      stop();
      return;
    }
    controlMode = ControlMode.POSITION;
    commandedVoltage = Volts.of(0.0);
    commandedVelocity = RotationsPerSecond.of(0.0);
    commandedPosition = position;
    io.setPosition(position);
  }

  public Angle getPosition() {
    return inputs.position;
  }

  public boolean isPositionControlReady() {
    return inputs.hardwareConfigurationValid;
  }

  public Angle getAbsolutePosition() {
    return inputs.absolutePosition;
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public Voltage getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public Current getSupplyCurrent() {
    return inputs.supplyCurrent;
  }

  public Current getStatorCurrent() {
    return inputs.statorCurrent;
  }

  public Angle getPositionError() {
    return Rotations.of(
        MathUtil.inputModulus(
            commandedPosition.in(Rotations) - inputs.position.in(Rotations), -0.5, 0.5));
  }

  public boolean atPosition(Angle tolerance) {
    return inputs.hardwareConfigurationValid
        && controlMode == ControlMode.POSITION
        && Math.abs(getPositionError().in(Rotations)) <= Math.abs(tolerance.in(Rotations));
  }

  /** True when the arm is high enough that the turntable may rotate under it. */
  public boolean isSafeToRotate() {
    return armHeight.get().in(Meters)
        >= CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
            + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters);
  }

  /** True once the turntable has stayed aligned with the slot for the settle time. */
  public boolean isSlotSettled(int slot) {
    checkSlotIndex(slot);
    if (getAlignedSlot().orElse(-1) != slot) {
      slotSettledSince = Double.NaN;
    } else if (!Double.isFinite(slotSettledSince)) {
      slotSettledSince = Timer.getTimestamp();
    }
    return Double.isFinite(slotSettledSince)
        && Timer.getTimestamp() - slotSettledSince
            >= CoralHandlingConstants.INDEXER_SETTLE_SECONDS;
  }

  /** Holds the turntable on the angle it was last commanded to. */
  public void holdCurrentPosition() {
    setPosition(commandedPosition);
  }

  /** Rotates to any angle by the short way; scan stops need no calibrated slot table. */
  public void rotateToAngle(Angle angle) {
    if (!inputs.hardwareConfigurationValid || !Double.isFinite(angle.in(Rotations))) {
      return;
    }
    setPosition(Rotations.of(
        inputs.position.in(Rotations) + angleDifference(angle, inputs.position)));
  }

  /**
   * True once the turntable has stayed within the given tolerance of the angle, under the
   * transfer speed limit, for the whole settle time. Judged live every cycle.
   */
  public boolean isAngleSettled(Angle target, Angle tolerance, double settleSeconds) {
    boolean atAngle = inputs.hardwareConfigurationValid && controlMode == ControlMode.POSITION
        && Math.abs(angleDifference(target, inputs.position))
            <= Math.abs(tolerance.in(Rotations))
        && Math.abs(inputs.velocity.in(RotationsPerSecond))
            <= SpinIndexerConstants.SLOT_TRANSFER_MAX_VELOCITY.in(RotationsPerSecond);
    if (!atAngle) {
      angleSettledSince = Double.NaN;
    } else if (!Double.isFinite(angleSettledSince)) {
      angleSettledSince = Timer.getTimestamp();
    }
    return Double.isFinite(angleSettledSince)
        && Timer.getTimestamp() - angleSettledSince >= settleSeconds;
  }

  /** Identifies a single aligned slot only when the turntable is sufficiently slow to transfer. */
  public OptionalInt getAlignedSlot() {
    if (!inputs.hardwareConfigurationValid
        || Math.abs(inputs.velocity.in(RotationsPerSecond))
            > SpinIndexerConstants.SLOT_TRANSFER_MAX_VELOCITY.in(RotationsPerSecond)) {
      return OptionalInt.empty();
    }
    for (int slot = 0; slot < SpinIndexerConstants.SLOT_COUNT; slot++) {
      if (Math.abs(angleDifference(slotAngle(slot), inputs.position))
          <= SpinIndexerConstants.SLOT_ALIGNMENT_TOLERANCE.in(Rotations)) {
        return OptionalInt.of(slot);
      }
    }
    return OptionalInt.empty();
  }

  /**
   * Requests Motion Magic alignment and leaves the position loop holding for the arm transfer.
   * The calling command must require this subsystem. Returns false without issuing a request
   * when the hardware configuration is unavailable.
   */
  public boolean rotateToSlot(int slot) {
    checkSlotIndex(slot);
    if (!inputs.hardwareConfigurationValid) {
      return false;
    }
    setPosition(
        Rotations.of(
            inputs.position.in(Rotations) + angleDifference(slotAngle(slot), inputs.position)));
    return true;
  }

  /** Shortest signed rotation, in turns, from the current position to the slot's transfer angle. */
  public double angleToSlot(int slot) {
    checkSlotIndex(slot);
    return angleDifference(slotAngle(slot), inputs.position);
  }

  public static Angle slotAngle(int slot) {
    return switch (slot) {
      case 0 -> SpinIndexerConstants.SLOT_1_ALIGNMENT_ANGLE;
      case 1 -> SpinIndexerConstants.SLOT_2_ALIGNMENT_ANGLE;
      case 2 -> SpinIndexerConstants.SLOT_3_ALIGNMENT_ANGLE;
      default -> throw new IndexOutOfBoundsException("SpinIndexer slot must be 0, 1, or 2: " + slot);
    };
  }

  private static void checkSlotIndex(int slot) {
    if (slot < 0 || slot >= SpinIndexerConstants.SLOT_COUNT) {
      throw new IndexOutOfBoundsException("SpinIndexer slot must be 0, 1, or 2: " + slot);
    }
  }

  private static double angleDifference(Angle target, Angle current) {
    return MathUtil.inputModulus(target.in(Rotations) - current.in(Rotations), -0.5, 0.5);
  }

  /** Plays a music tone through the spin indexer motor. */
  public boolean setMusicTone(Frequency frequency) {
    if (!inputs.hardwareConfigurationValid) {
      stop();
      return false;
    }
    controlMode = ControlMode.STOPPED;
    commandedVoltage = Volts.of(0.0);
    commandedVelocity = RotationsPerSecond.of(0.0);
    commandedPosition = inputs.position;
    boolean succeeded = io.setMusicTone(frequency);
    Logger.recordOutput("SpinIndexer/MusicToneRequestSucceeded", succeeded);
    return succeeded;
  }

  /** Stops music output and neutralizes the spin indexer motor. */
  public void stopMusicTone() {
    io.stopMusicTone();
  }


  public void stop() {
    controlMode = ControlMode.STOPPED;
    commandedVoltage = Volts.of(0.0);
    commandedVelocity = RotationsPerSecond.of(0.0);
    commandedPosition = inputs.position;
    io.stop();
  }

  // ========== SysId ==========

  public void runCurrentCharacterization(double currentAmps) {
    if (!Double.isFinite(currentAmps) || !inputs.hardwareConfigurationValid) {
      stop();
      return;
    }
    controlMode = ControlMode.TORQUE_CURRENT;
    commandedCurrentAmps = currentAmps;
    commandedVoltage = Volts.zero();
    commandedVelocity = RotationsPerSecond.zero();
    commandedPosition = inputs.position;
    io.setTorqueCurrent(Amps.of(currentAmps));
  }


  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction).withTimeout(SpinIndexerConstants.SYSID_DYNAMIC_DURATION);
  }

  // ========== Command Factories ==========

  public Command runVoltageCommand(Supplier<Voltage> voltage) {
    BooleanSupplier isFinished = () -> false;
    return run(
            c -> {
              Logger.recordOutput("Commands/SpinIndexerVoltage/CommandActive", true);
              while (!isFinished.getAsBoolean()) {
                setVoltage(voltage.get());
                c.yield();
              }
            })
        .whenCanceled(
            () -> {
              stop();
              Logger.recordOutput("Commands/SpinIndexerVoltage/CommandActive", false);
            })
        .named("SpinIndexer:RunVoltageCommand");
  }

  public Command runVelocityCommand(Supplier<AngularVelocity> velocity) {
    BooleanSupplier isFinished = () -> false;
    return run(
            c -> {
              Logger.recordOutput("Commands/SpinIndexerVelocity/CommandActive", true);
              while (!isFinished.getAsBoolean()) {
                setVelocity(velocity.get());
                c.yield();
              }
            })
        .whenCanceled(
            () -> {
              stop();
              Logger.recordOutput("Commands/SpinIndexerVelocity/CommandActive", false);
            })
        .named("SpinIndexer:RunVelocityCommand");
  }

  public Command runPositionCommand(Supplier<Angle> position) {
    BooleanSupplier isFinished = () -> false;
    return run(
            c -> {
              Logger.recordOutput("Commands/SpinIndexerPosition/CommandActive", true);
              while (!isFinished.getAsBoolean()) {
                setPosition(position.get());
                c.yield();
              }
            })
        .whenCanceled(
            () -> {
              stop();
              Logger.recordOutput("Commands/SpinIndexerPosition/CommandActive", false);
            })
        .named("SpinIndexer:RunPositionCommand");
  }
}
