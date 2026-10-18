// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    // TODO: Verify 50 Hz yaw-rate feedback remains timely under full CAN load.
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    var odometryThread = PhoenixOdometryThread.getInstance();
    yawPositionQueue = odometryThread.registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRotPerSec = yawVelocity.getValueAsDouble() / 360.0;
    double yawAge = yaw.getTimestamp().getLatency();
    double rateAge = yawVelocity.getTimestamp().getLatency();
    inputs.motionFeedbackAgeSeconds = yawAge >= 0 && rateAge >= 0
        ? Math.max(yawAge, rateAge) : Double.NaN;
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map(Rotation2d::fromDegrees)
            .toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}
