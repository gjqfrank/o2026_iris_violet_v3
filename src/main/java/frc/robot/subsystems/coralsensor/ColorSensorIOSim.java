// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import org.wpilib.system.Timer;
import frc.robot.subsystems.spinindexer.CoralState;

/** Ideal scene-generated readings; raw readings remain available for manual sensor experiments. */
public final class ColorSensorIOSim implements ColorSensorIO {
  private final int whiteChannel;
  private int red = -1;
  private int green = -1;
  private int blue = -1;
  private boolean connected;
  private double sampledAt = Double.NaN;

  public ColorSensorIOSim() {
    this(CoralSensorConstants.COLOR_ADC_MAX);
  }

  /** Each mount can use a different assumed optical strength. */
  public ColorSensorIOSim(int whiteChannel) { this.whiteChannel = whiteChannel; }

  public void setCoralReading(CoralState color, boolean connected) {
    // Presence is independently simulated by CANrangeIOSim.
    int channel = color == CoralState.WHITE ? whiteChannel : 0;
    setRawReading(channel, channel, channel, connected);
  }

  public void setRawReading(int red, int green, int blue, boolean connected) {
    this.red = red;
    this.green = green;
    this.blue = blue;
    this.connected = connected;
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    if (!connected) {
      return;
    }
    double now = Timer.getTimestamp();
    if (Double.isFinite(sampledAt) && now >= sampledAt && now - sampledAt < 0.1) return;
    // Match the real IO's configured 100 ms measurement period.
    sampledAt = now;
    inputs.red = red;
    inputs.green = green;
    inputs.blue = blue;
    inputs.colorTimestampSeconds = now;
  }
}
