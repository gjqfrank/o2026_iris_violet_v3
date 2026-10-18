// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.toggles;

import org.wpilib.simulation.DIOSim;

/** Simulation implementation of TogglesIO */
public class TogglesIOSim implements TogglesIO {
  private final DIOSim brakeModeButtonSim = new DIOSim(ToggleConstants.BRAKE_MODE_BUTTON_DIO);
  private final DIOSim homeButtonSim = new DIOSim(ToggleConstants.HOME_BUTTON_DIO);

  public TogglesIOSim() {
    // Configure as inputs
    brakeModeButtonSim.setIsInput(true);
    homeButtonSim.setIsInput(true);

    // Set default values (not pressed, active-low)
    brakeModeButtonSim.setValue(true);
    homeButtonSim.setValue(true);
  }

  @Override
  public void updateInputs(TogglesIOInputs inputs) {
    inputs.brakeModeButtonPressed = !brakeModeButtonSim.getValue();
    inputs.homeButtonPressed = !homeButtonSim.getValue();
  }

  /** Simulate pressing the brake mode button */
  public void setBrakeModeButtonPressed(boolean pressed) {
    brakeModeButtonSim.setValue(!pressed); // Active-low
  }

  /** Simulate pressing the home button */
  public void setHomeButtonPressed(boolean pressed) {
    homeButtonSim.setValue(!pressed); // Active-low
  }
}
