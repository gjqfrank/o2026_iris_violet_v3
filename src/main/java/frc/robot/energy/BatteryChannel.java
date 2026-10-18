// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.energy;

import static frc.robot.energy.BatteryAggregate.*;

/** Motor/device leaf channel for {@link BatteryLogger}. Main robot thread only. */
public enum BatteryChannel {
  DRIVE_MODULE0_DRIVE("Drive/Module0/Drive", DRIVE_MODULE0, DRIVE),
  DRIVE_MODULE0_TURN("Drive/Module0/Turn", DRIVE_MODULE0, DRIVE),
  DRIVE_MODULE1_DRIVE("Drive/Module1/Drive", DRIVE_MODULE1, DRIVE),
  DRIVE_MODULE1_TURN("Drive/Module1/Turn", DRIVE_MODULE1, DRIVE),
  DRIVE_MODULE2_DRIVE("Drive/Module2/Drive", DRIVE_MODULE2, DRIVE),
  DRIVE_MODULE2_TURN("Drive/Module2/Turn", DRIVE_MODULE2, DRIVE),
  DRIVE_MODULE3_DRIVE("Drive/Module3/Drive", DRIVE_MODULE3, DRIVE),
  DRIVE_MODULE3_TURN("Drive/Module3/Turn", DRIVE_MODULE3, DRIVE),

  ELEVATOR_LEADER("Elevator/Leader", ELEVATOR),
  ELEVATOR_FOLLOWER("Elevator/Follower", ELEVATOR),
  SPININDEXER_MOTOR("SpinIndexer/Motor", SPININDEXER),
  END_EFFECTOR_COMPRESSOR("EndEffector/Compressor", END_EFFECTOR);

  final String currentKey;
  final String powerKey;
  final String energyKey;
  final BatteryAggregate[] parents;

  BatteryChannel(String path, BatteryAggregate... parents) {
    String prefix = "Power/BatteryEstimate/Debug/Channels/" + path;
    this.currentKey = prefix + "/ReportedSupplyCurrentAmps";
    this.powerKey = prefix + "/EstimatedPowerWatts";
    this.energyKey = prefix + "/EstimatedEnergyJoules";
    this.parents = parents;
  }

  /** Explicit drive-motor channel for a swerve module index 0-3. */
  public static BatteryChannel driveChannel(int moduleIndex) {
    return switch (moduleIndex) {
      case 0 -> DRIVE_MODULE0_DRIVE;
      case 1 -> DRIVE_MODULE1_DRIVE;
      case 2 -> DRIVE_MODULE2_DRIVE;
      case 3 -> DRIVE_MODULE3_DRIVE;
      default -> throw new IndexOutOfBoundsException("Drive module index must be 0-3: " + moduleIndex);
    };
  }

  /** Explicit turn-motor channel for a swerve module index 0-3. */
  public static BatteryChannel turnChannel(int moduleIndex) {
    return switch (moduleIndex) {
      case 0 -> DRIVE_MODULE0_TURN;
      case 1 -> DRIVE_MODULE1_TURN;
      case 2 -> DRIVE_MODULE2_TURN;
      case 3 -> DRIVE_MODULE3_TURN;
      default -> throw new IndexOutOfBoundsException("Drive module index must be 0-3: " + moduleIndex);
    };
  }
}
