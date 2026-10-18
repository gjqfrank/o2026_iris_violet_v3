// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.endeffector;

public final class EndEffectorConstants {
  public static final boolean REAL_IO_CONFIGURED = false; // TODO: Enable after PH wiring, valve voltage and jaw direction are verified.
  public static final int PH_CAN_ID = -1; // TODO: Set the REV PH CAN ID on the roboRIO CAN bus (1-63).
  public static final int FORWARD_CHANNEL = -1; // TODO: Set the first valve coil's PH channel (0-15).
  public static final int REVERSE_CHANNEL = -1; // TODO: Set the second valve coil's PH channel (0-15), distinct from FORWARD_CHANNEL.
  // 1: forward opens the jaw; 0: reverse opens the jaw; -1: not configured.
  public static final int OPEN_USES_FORWARD = -1; // TODO: Coil energized to open the jaw: 1 = forward, 0 = reverse, -1 = unconfigured; verify opening direction with the actual air plumbing.

  private EndEffectorConstants() {}

}
