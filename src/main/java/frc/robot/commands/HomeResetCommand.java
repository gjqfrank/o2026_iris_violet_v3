// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.commands;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Milliseconds;

import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;
import org.wpilib.command3.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.spinindexer.SpinIndexer;
import org.littletonrobotics.junction.Logger;

/** Manually calibrates elevator height without changing pivot's absolute reference. */
public final class HomeResetCommand {
  private static final Frequency FIRST_NOTE = Hertz.of(880.00);
  private static final Frequency SECOND_NOTE = Hertz.of(1174.66);
  private static final Frequency THIRD_NOTE = Hertz.of(1567.98);
  private static final Time SHORT_NOTE_DURATION = Milliseconds.of(90.0);
  private static final Time NOTE_GAP_DURATION = Milliseconds.of(40.0);
  private static final Time FINAL_NOTE_DURATION = Milliseconds.of(180.0);

  private HomeResetCommand() {}

  /**
   * Resets only elevator height, then plays the success tone through drive, spin indexer, and
   * elevator motors.
   *
   * <p>Only run while disabled and stationary, with the elevator physically at its home height.
   */
  public static Command create(Drive drive, SpinIndexer spinIndexer, Elevator elevator) {
    return Command.requiring(drive, spinIndexer, elevator)
        .executing(
            coroutine -> {
              boolean resetSucceeded = elevator.resetHeight(ElevatorConstants.HOME_POSITION);
              Logger.recordOutput("Commands/HomeReset/ElevatorSucceeded", resetSucceeded);
              if (!resetSucceeded) {
                System.out.println("Elevator home reset failed; no success tone");
                return;
              }

              System.out.println("Elevator home reset succeeded");
              setTone(drive, spinIndexer, elevator, FIRST_NOTE);
              coroutine.wait(SHORT_NOTE_DURATION);
              setTone(drive, spinIndexer, elevator, Hertz.zero());
              coroutine.wait(NOTE_GAP_DURATION);
              setTone(drive, spinIndexer, elevator, SECOND_NOTE);
              coroutine.wait(SHORT_NOTE_DURATION);
              setTone(drive, spinIndexer, elevator, Hertz.zero());
              coroutine.wait(NOTE_GAP_DURATION);
              setTone(drive, spinIndexer, elevator, THIRD_NOTE);
              coroutine.wait(FINAL_NOTE_DURATION);
            })
        .whenExited(
            () -> {
              drive.stopMusicTone();
              spinIndexer.stopMusicTone();
              elevator.stopMusicTone();
            })
        .named("HomeReset");
  }

  private static void setTone(
      Drive drive, SpinIndexer spinIndexer, Elevator elevator, Frequency frequency) {
    boolean driveRequestSucceeded = drive.setMusicTone(frequency);
    boolean spinIndexerRequestSucceeded = spinIndexer.setMusicTone(frequency);
    boolean elevatorRequestSucceeded = elevator.setMusicTone(frequency);
    Logger.recordOutput(
        "Commands/HomeReset/ToneRequestSucceeded",
        driveRequestSucceeded && spinIndexerRequestSucceeded && elevatorRequestSucceeded);
  }
}
