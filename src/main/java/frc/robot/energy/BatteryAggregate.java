// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.energy;

/** Parent aggregate nodes for {@link BatteryLogger}. Main robot thread only. */
enum BatteryAggregate {
  DRIVE("Drive"),
  DRIVE_MODULE0("Drive/Module0"),
  DRIVE_MODULE1("Drive/Module1"),
  DRIVE_MODULE2("Drive/Module2"),
  DRIVE_MODULE3("Drive/Module3"),
  ELEVATOR("Elevator"),
  SPININDEXER("SpinIndexer"),
  END_EFFECTOR("EndEffector");

  final String currentKey;
  final String powerKey;
  final String energyKey;

  BatteryAggregate(String path) {
    String prefix = "Power/BatteryEstimate/Debug/Aggregates/" + path;
    currentKey = prefix + "/ReportedSupplyCurrentAmps";
    powerKey = prefix + "/EstimatedPowerWatts";
    energyKey = prefix + "/EstimatedEnergyJoules";
  }
}
