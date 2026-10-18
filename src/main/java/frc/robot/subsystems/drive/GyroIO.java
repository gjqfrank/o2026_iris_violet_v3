// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public double motionFeedbackAgeSeconds = Double.NaN;
    public Rotation2d yawPosition = Rotation2d.ZERO;
    public double yawVelocityRotPerSec = 0.0;
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
