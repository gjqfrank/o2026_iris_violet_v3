// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.common;

/** Shared control mode enum for all subsystems. Logs use the readable enum name. */
public enum ControlMode {
  STOPPED(0),
  VOLTAGE(1),
  VELOCITY(2),
  POSITION(3),
  TORQUE_CURRENT(4);

  private final int value;

  ControlMode(int value) {
    this.value = value;
  }

  public int getValue() {
    return value;
  }
}
