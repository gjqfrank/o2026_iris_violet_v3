// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.toggles;

import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.util.function.BooleanConsumer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Toggles subsystem for managing physical buttons */
public class Toggles implements Mechanism {
  private final TogglesIO io;
  private final TogglesIOInputsAutoLogged inputs = new TogglesIOInputsAutoLogged();

  public Toggles(TogglesIO io) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    this.io = io;
  }

  private void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Toggles", inputs);

    Logger.recordOutput("Toggles/TOGGLE_HOME", inputs.homeButtonPressed);
    Logger.recordOutput("Toggles/TOGGLE_BRAKE", inputs.brakeModeButtonPressed);
  }

  /** Returns true if the brake mode button is currently pressed */
  public boolean isBrakeModeButtonPressed() {
    return inputs.brakeModeButtonPressed;
  }

  /** Returns true if the home button is currently pressed */
  public boolean isHomeButtonPressed() {
    return inputs.homeButtonPressed;
  }

  public static void createDebouncedTriggerOnChange(
      BooleanSupplier rawButtonSupplier, BooleanConsumer onButtonChange) {
    new Trigger(rawButtonSupplier)
        .debounce(org.wpilib.units.Units.Seconds.of(0.05), DebounceType.BOTH)
        // TODO: Validate 50 ms brake-button debounce against switch bounce and response time.
        .onTrue(
            Command.noRequirements(c -> onButtonChange.accept(true)).named("Toggles:OnChangeTrue"))
        .onFalse(
            Command.noRequirements(c -> onButtonChange.accept(false)).named("Toggles:OnChangeFalse"));
  }

  public static void createDebouncedTriggerOnTrue(
      BooleanSupplier rawButtonSupplier, Command onButtonTrue) {
    // TODO: Validate 50 ms home-button rising debounce against switch bounce and unintended resets.
    new Trigger(rawButtonSupplier)
        .debounce(org.wpilib.units.Units.Seconds.of(0.05), DebounceType.RISING)
        .onTrue(onButtonTrue);
  }
}
