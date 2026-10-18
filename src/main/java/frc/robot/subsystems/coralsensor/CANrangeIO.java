// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {
  @AutoLog
  class CANrangeIOInputs {
    /** Configuration write result, independent of sensor connection state. */
    public boolean configurationValid = false;
    public Distance distance = Meters.of(Double.NaN);
    public boolean detected = false;
    /** Advances only for new measurements, in replayable robot time, seconds. */
    public double measurementTimestampSeconds = Double.NaN;
  }

  default void updateInputs(CANrangeIOInputs inputs) {}
  default void setDetectionConfig(Distance threshold, Distance hysteresis, double minSignalStrength) {}
}
