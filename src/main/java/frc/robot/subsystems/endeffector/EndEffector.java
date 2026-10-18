// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.endeffector;

import static org.wpilib.units.Units.Amps;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import frc.robot.constants.Constants;
import frc.robot.energy.BatteryChannel;
import frc.robot.energy.BatteryLogger;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

/** Gripper pneumatic control only; coral sensing lives in the independent CoralSensor. */
public class EndEffector implements Mechanism {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private final Alert hardwareUnconfigured =
      new Alert("hardware unconfigured","EndEffector hardware is not configured; real IO disabled.", Level.MEDIUM);
  private final Alert solenoidFault =
      new Alert("solenoidFault","EndEffector solenoid channel disabled by PH fault.", Level.MEDIUM);

  public EndEffector(EndEffectorIO io) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    this.io = Objects.requireNonNull(io);
  }

  private void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    hardwareUnconfigured.set(
        Constants.getMode() == Constants.Mode.REAL && !inputs.hardwareConfigurationValid);
    solenoidFault.set(
        inputs.forwardChannelFaultDisabled || inputs.reverseChannelFaultDisabled);
    BatteryLogger.getInstance()
        .reportSupplyCurrent(
            BatteryChannel.END_EFFECTOR_COMPRESSOR,
            inputs.compressorCurrentAvailable ? inputs.compressorCurrent.in(Amps) : 0.0);
    // No implicit jaw movement or compressor mode change on enable/disable transitions.
  }

  /** Commands the valve only; sensor observations do not confirm jaw arrival or retention. */
  public void openGripper() {
    setGripperOpen(true);
  }

  /** Commands the valve only; sensor observations do not confirm jaw arrival or retention. */
  public void clampGripper() {
    setGripperOpen(false);
  }

  private void setGripperOpen(boolean open) {
    if (RobotState.isDisabled()) return;
    io.setGripperOpen(open);
  }

  public void enableAutomaticCompressor() {
    io.setAutomaticCompressorEnabled(true);
  }

  /** Stops air generation; does not vent the tank or change the gripper valve. */
  public void disableAutomaticCompressor() {
    io.setAutomaticCompressorEnabled(false);
  }

  /** One-shot commands own this subsystem but do not wait for cylinder travel. */
  public Command openCommand() {
    return run(c -> openGripper()).named("EndEffector Open");
  }

  public Command clampCommand() {
    return run(c -> clampGripper()).named("EndEffector Clamp");
  }

  public Command enableAutomaticCompressorCommand() {
    return run(c -> enableAutomaticCompressor()).named("EndEffector Enable Automatic Air");
  }

  public Command disableAutomaticCompressorCommand() {
    return run(c -> disableAutomaticCompressor()).named("EndEffector Disable Automatic Air");
  }

  /** Valve electrical output readback, not the physical jaw state. */
  public String getValveOutput() {
    return inputs.valveOutput;
  }

  /** True when the valve was last commanded open; not the physical jaw state. */
  public boolean isGripperOpen() {
    return "OPEN".equals(inputs.valveOutput);
  }

  /** True when the valve was last commanded clamped; not the physical jaw state. */
  public boolean isGripperClamped() {
    return "CLAMP".equals(inputs.valveOutput);
  }

  public boolean isGripperReady() {
    return inputs.hardwareConfigurationValid
        && !inputs.forwardChannelFaultDisabled && !inputs.reverseChannelFaultDisabled;
  }
}
