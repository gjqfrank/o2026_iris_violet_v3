// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.toggles;

public final class ToggleConstants {
  // Confirm operator panel DIO channels and active polarity from the electrical layout before
  // enabling real IO. Placeholder DIO channels are intentionally not deployable.
  public static final boolean REAL_IO_CONFIGURED = false; // TODO: Enable after DIO channels and active polarity are verified.
  public static final int BRAKE_MODE_BUTTON_DIO =
      -1; // TODO: Update to actual DIO channel for brake mode button
  public static final int HOME_BUTTON_DIO =
      -1; // TODO: Update to actual DIO channel for home button
  public static final boolean BRAKE_MODE_BUTTON_ACTIVE_LOW = false; // TODO: Verify brake button active polarity against wiring.
  public static final boolean HOME_BUTTON_ACTIVE_LOW = false; // TODO: Verify home button active polarity against wiring.

  private ToggleConstants() {}

}
