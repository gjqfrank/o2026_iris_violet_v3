// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import static org.wpilib.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.system.Timer;

/** Phoenix 26.3 CANrange IsDetected applies distance hysteresis and signal-strength filtering. */
public final class CANrangeIOPhoenix implements CANrangeIO, AutoCloseable {
  private final CANrange sensor;
  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> detected;
  private final StatusSignal<Time> measurementTime;
  private boolean configurationValid;
  private double lastMeasurementTime = Double.NaN;

  public CANrangeIOPhoenix(int canId, String canBusName) {
    sensor = new CANrange(canId, new CANBus(canBusName));
    distance = sensor.getDistance();
    detected = sensor.getIsDetected();
    measurementTime = sensor.getMeasurementTime();
    // TODO: Verify 50 Hz CAN status delivery and the firmware's ToF/FOV settings at both mounts.
    BaseStatusSignal.setUpdateFrequencyForAll(50, distance, detected, measurementTime);
    sensor.optimizeBusUtilization();
  }

  @Override
  public void setDetectionConfig(Distance threshold, Distance hysteresis, double minSignalStrength) {
    ProximityParamsConfigs config = new ProximityParamsConfigs();
    config.ProximityThreshold = threshold.in(Meters);
    config.ProximityHysteresis = hysteresis.in(Meters);
    config.MinSignalStrengthForValidMeasurement = minSignalStrength;
    configurationValid = tryUntilOk(5, () -> sensor.getConfigurator().apply(config, 0.25)).isOK();
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    inputs.configurationValid = configurationValid;
    // Failed reads supply no new sample; retain prior readings/timestamps, like the RGB IO.
    if (!BaseStatusSignal.refreshAll(distance, detected, measurementTime).isOK()) return;
    double sampleTime = measurementTime.getValue().in(Seconds);
    if (sampleTime == lastMeasurementTime) return;
    lastMeasurementTime = sampleTime;
    inputs.distance = distance.getValue();
    inputs.detected = detected.getValue();
    // Device MeasurementTime wraps and is not synchronized to the robot clock.
    inputs.measurementTimestampSeconds = Timer.getTimestamp();
  }

  @Override
  public void close() { sensor.close(); }
}
