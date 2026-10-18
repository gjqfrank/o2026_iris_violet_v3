// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.toggles;

import org.wpilib.hardware.discrete.DigitalInput;

/** Hardware implementation of TogglesIO */
public class TogglesIOHardware implements TogglesIO {
  private final DigitalInput brakeModeButton =
      new DigitalInput(ToggleConstants.BRAKE_MODE_BUTTON_DIO);
  private final DigitalInput homeButton = new DigitalInput(ToggleConstants.HOME_BUTTON_DIO);

  @Override
  public void updateInputs(TogglesIOInputs inputs) {
    inputs.brakeModeButtonPressed =
        readPressed(brakeModeButton, ToggleConstants.BRAKE_MODE_BUTTON_ACTIVE_LOW);
    inputs.homeButtonPressed = readPressed(homeButton, ToggleConstants.HOME_BUTTON_ACTIVE_LOW);
  }

  private static boolean readPressed(DigitalInput input, boolean activeLow) {
    boolean raw = input.get();
    return activeLow ? !raw : raw;
  }
}
