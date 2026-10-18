// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.Distance;
import org.wpilib.system.Timer;

/** Scene-provided physical distance; detection uses the same threshold and hysteresis as REAL. */
public final class CANrangeIOSim implements CANrangeIO {
  private Distance distance = Meters.of(Double.NaN);
  private double signalStrength;
  private boolean connected;
  private double threshold;
  private double hysteresis;
  private double minSignalStrength;
  private boolean configurationValid;
  private boolean detected;
  private double sampledAt = Double.NaN;

  public void setReading(Distance distance, double signalStrength, boolean connected) {
    this.distance = distance;
    this.signalStrength = signalStrength;
    this.connected = connected;
  }

  @Override
  public void setDetectionConfig(Distance threshold, Distance hysteresis, double minSignalStrength) {
    this.threshold = threshold.in(Meters);
    this.hysteresis = hysteresis.in(Meters);
    this.minSignalStrength = minSignalStrength;
    configurationValid = true;
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    inputs.configurationValid = configurationValid;
    if (!connected) return;
    double now = Timer.getTimestamp();
    if (now == sampledAt) return;
    sampledAt = now;
    double detectionLimit = threshold + (detected ? hysteresis : -hysteresis);
    detected = signalStrength >= minSignalStrength && Double.isFinite(distance.in(Meters))
        && distance.in(Meters) <= detectionLimit;
    inputs.distance = distance;
    inputs.detected = detected;
    inputs.measurementTimestampSeconds = now;
  }
}
