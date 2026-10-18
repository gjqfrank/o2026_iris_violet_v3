// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.endeffector;

import org.wpilib.system.Timer;

/** Exposes valve requests to CoralSceneSim; pressure and compressor load are not modeled. */
public class EndEffectorIOSim implements EndEffectorIO {
  private String valveOutput = "OFF";
  private boolean automaticCompressorEnabled = true;
  public record GripperRequest(boolean open, double timestamp, long sequence) {}
  private GripperRequest gripperRequest;
  private long requestSequence;

  public GripperRequest getGripperRequest() {
    return gripperRequest;
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.hardwareConfigurationValid = true;
    inputs.compressorCurrentAvailable = true; // Zero modeled load; no pneumatic plant is simulated.
    inputs.valveOutput = valveOutput;
    inputs.compressorControlMode = automaticCompressorEnabled ? "Digital" : "Disabled";
    // Running/current/pressure-switch remain at their defaults without a pneumatic plant model.
  }

  @Override
  public void setGripperOpen(boolean open) {
    valveOutput = open ? "OPEN" : "CLAMP";
    // Count repeated requests too: closing an already-closed empty jaw is still an attempt.
    gripperRequest = new GripperRequest(open, Timer.getTimestamp(), ++requestSequence);
  }

  @Override
  public void setAutomaticCompressorEnabled(boolean enabled) {
    automaticCompressorEnabled = enabled;
  }
}
