// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.endeffector;

import static org.wpilib.units.Units.Amps;

import org.wpilib.hardware.pneumatic.Compressor;
import org.wpilib.hardware.pneumatic.DoubleSolenoid;
import org.wpilib.hardware.pneumatic.DoubleSolenoid.Value;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;
import org.wpilib.hardware.bus.CANPort;

/** Owns the only pneumatic mechanism and compressor on the robot. */
public class EndEffectorIOHardware implements EndEffectorIO, AutoCloseable {
  private final DoubleSolenoid gripper;
  private final Compressor compressor;

  public EndEffectorIOHardware() {
    if (!EndEffectorConstants.REAL_IO_CONFIGURED) {
      throw new IllegalStateException("EndEffector real IO is disabled.");
    }
    gripper = new DoubleSolenoid(
      CANPort.CAN_S0,
        EndEffectorConstants.PH_CAN_ID,
        PneumaticsModuleType.REV_PH,
        EndEffectorConstants.FORWARD_CHANNEL,
        EndEffectorConstants.REVERSE_CHANNEL);
    try {
      // WPILib 2026.2.1 enables digital closed-loop regulation in this constructor.
      // No periodic/Disabled callback rewrites the selected compressor mode.
      compressor = new Compressor(CANPort.CAN_S0, EndEffectorConstants.PH_CAN_ID, PneumaticsModuleType.REV_PH);
    } catch (RuntimeException exception) {
      gripper.close();
      throw exception;
    }
    // Do not select OPEN or CLAMP on startup; the physical jaw position is unknown.
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.hardwareConfigurationValid = true;
    Value output = gripper.get();
    inputs.valveOutput = output == Value.OFF
        ? "OFF" : output == openValue() ? "OPEN" : "CLAMP";
    inputs.compressorControlMode = compressor.getConfigType().name();
    inputs.compressorRunning = compressor.isEnabled();
    inputs.pressureSwitchNeedsAir = compressor.getPressureSwitchValue();
    inputs.compressorCurrent = Amps.of(compressor.getCurrent());
    inputs.compressorCurrentAvailable = Double.isFinite(inputs.compressorCurrent.in(Amps));
    inputs.forwardChannelFaultDisabled = gripper.isFwdSolenoidDisabled();
    inputs.reverseChannelFaultDisabled = gripper.isRevSolenoidDisabled();
  }

  @Override
  public void setGripperOpen(boolean open) {
    gripper.set(open ? openValue() : clampValue());
  }

  @Override
  public void setAutomaticCompressorEnabled(boolean enabled) {
    if (enabled) {
      compressor.enableDigital();
    } else {
      compressor.disable();
    }
  }

  private static Value openValue() {
    return EndEffectorConstants.OPEN_USES_FORWARD == 1 ? Value.FORWARD : Value.REVERSE;
  }

  private static Value clampValue() {
    return EndEffectorConstants.OPEN_USES_FORWARD == 1 ? Value.REVERSE : Value.FORWARD;
  }

  @Override
  public void close() {
    compressor.disable();
    gripper.close();
    compressor.close();
  }
}
