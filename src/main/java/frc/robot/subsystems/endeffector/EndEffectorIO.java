// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.endeffector;

import static org.wpilib.units.Units.Amps;

import org.wpilib.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  class EndEffectorIOInputs {
    /** Configuration validity only, not PH connection or physical jaw feedback. */
    public boolean hardwareConfigurationValid = false;
    /** UNKNOWN, OFF, OPEN or CLAMP; describes valve electrical output, not jaw arrival. */
    public String valveOutput = "UNKNOWN";
    public String compressorControlMode = "UNKNOWN";
    public boolean compressorRunning = false;
    /** True means the digital switch requests air; it does not provide pressure in PSI. */
    public boolean pressureSwitchNeedsAir = false;
    public Current compressorCurrent = Amps.zero();
    /** A current reading was supplied by IO; not a PH connection or pneumatic pressure check. */
    public boolean compressorCurrentAvailable = false;
    /** PH short-circuit blacklist flags, independent of Driver Station Disabled. */
    public boolean forwardChannelFaultDisabled = false;
    public boolean reverseChannelFaultDisabled = false;
  }

  default void updateInputs(EndEffectorIOInputs inputs) {}

  /** Selects one valve coil; the subsystem rejects Disabled requests before calling this method. */
  default void setGripperOpen(boolean open) {}

  /** Enables digital pressure-switch regulation, or disables compressor operation. */
  default void setAutomaticCompressorEnabled(boolean enabled) {}
}
