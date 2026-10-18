// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import static org.wpilib.units.Units.Meters;

import frc.robot.subsystems.coralsensor.CoralSensorConstants.SensorConfig;
import frc.robot.util.LoggedTunableNumber;

/** Instance fields keep the gripper and indexer sensor parameters independent. */
final class CoralSensorTuning {
  final LoggedTunableNumber thresholdMeters;
  final LoggedTunableNumber hysteresisMeters;
  final LoggedTunableNumber minSignalStrength;
  final LoggedTunableNumber blackBrightnessMax;
  final LoggedTunableNumber whiteBrightnessMin;

  CoralSensorTuning(String logPath, SensorConfig config) {
    thresholdMeters = new LoggedTunableNumber(logPath + "/PresenceThresholdMeters", config.presenceThreshold().in(Meters));
    hysteresisMeters = new LoggedTunableNumber(logPath + "/PresenceHysteresisMeters", config.presenceHysteresis().in(Meters));
    minSignalStrength = new LoggedTunableNumber(logPath + "/MinSignalStrength", config.minSignalStrength());
    blackBrightnessMax = new LoggedTunableNumber(logPath + "/BlackBrightnessMax", config.blackBrightnessMax());
    whiteBrightnessMin = new LoggedTunableNumber(logPath + "/WhiteBrightnessMin", config.whiteBrightnessMin());
  }

  void update(int callerId, CANrangeIO rangeIO, Runnable resetObservation) {
    LoggedTunableNumber.ifChanged(callerId, () -> {
      rangeIO.setDetectionConfig(
          Meters.of(thresholdMeters.get()), Meters.of(hysteresisMeters.get()), minSignalStrength.get());
      resetObservation.run();
    }, thresholdMeters, hysteresisMeters, minSignalStrength);
    LoggedTunableNumber.ifChanged(callerId, resetObservation, blackBrightnessMax, whiteBrightnessMin);
  }

}
