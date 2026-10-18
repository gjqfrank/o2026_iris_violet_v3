// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.toggles;

import org.littletonrobotics.junction.AutoLog;

public interface TogglesIO {
  @AutoLog
  public static class TogglesIOInputs {
    public boolean brakeModeButtonPressed = false; // Locking button for brake/coast mode
    public boolean homeButtonPressed = false; // Momentary button for home function
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TogglesIOInputs inputs) {}
}
