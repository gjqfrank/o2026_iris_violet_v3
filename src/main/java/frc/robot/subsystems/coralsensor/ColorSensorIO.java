// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import org.littletonrobotics.junction.AutoLog;

/** Raw RGB observations; object presence comes exclusively from the paired CANrange. */
public interface ColorSensorIO {
  @AutoLog
  class ColorSensorIOInputs {
    public int red = -1;
    public int green = -1;
    public int blue = -1;
    /** Updated only for new sensor measurements, in the robot's replayable time base, seconds. */
    public double colorTimestampSeconds = Double.NaN;
  }

  default void updateInputs(ColorSensorIOInputs inputs) {}
}
