package frc.robot.commands;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.system.Timer;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorHomingCommand implements Command{
    private final Elevator elevator;
    private final Timer timeout = new Timer();
    private Debouncer stall;
    private boolean detectedStall;
    private boolean failed;

    public ElevatorHomingCommand(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void run(Coroutine c){
    detectedStall = false;
    failed = false;
    timeout.restart();
    Logger.recordOutput("Commands/ElevatorHoming/CommandActive", true);
    Logger.recordOutput("Commands/ElevatorHoming/ResetSucceeded", false);
    Logger.recordOutput("Commands/ElevatorHoming/FailureReason", "");
    if (!elevator.beginHoming()) {
      fail("Unable to start homing");
      return;
    }
    stall = new Debouncer(ElevatorConstants.HOMING_STALL_TIME.in(Seconds), DebounceType.RISING);
    stall.calculate(false);
    while(!isFinished()){
        if (timeout.hasElapsed(ElevatorConstants.HOMING_TIMEOUT.in(Seconds))) {
      fail("Homing timed out");
      return;
    }
    if (!elevator.setHomingVoltage(ElevatorConstants.HOMING_VOLTAGE)) {
      fail("Homing control rejected");
      return;
    }
    boolean currentsHigh =
        Math.abs(elevator.getLeaderStatorCurrent().in(Amps))
            >= ElevatorConstants.HOMING_CURRENT_THRESHOLD.in(Amps)
        && Math.abs(elevator.getFollowerStatorCurrent().in(Amps))
            >= ElevatorConstants.HOMING_CURRENT_THRESHOLD.in(Amps);
    boolean stationary =
        Math.abs(elevator.getHeightVelocity().in(MetersPerSecond))
            <= ElevatorConstants.HOMING_VELOCITY_THRESHOLD.in(MetersPerSecond)
        && Math.abs(elevator.getPivotVelocity().in(RadiansPerSecond))
            <= ElevatorConstants.RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond);
    detectedStall = stall.calculate(
        timeout.hasElapsed(ElevatorConstants.HOMING_START_DELAY.in(Seconds))
            && currentsHigh && stationary && elevator.isHomingHealthy());
    Logger.recordOutput("Commands/ElevatorHoming/CurrentsHigh", currentsHigh);
    Logger.recordOutput("Commands/ElevatorHoming/Stationary", stationary);
    Logger.recordOutput("Commands/ElevatorHoming/StallDetected", detectedStall);

    c.yield();
    }
    end(false);
  }

  private void fail(String reason) {
    failed = true;
    elevator.stop();
    Logger.recordOutput("Commands/ElevatorHoming/FailureReason", reason);
  }

  public boolean isFinished() { return detectedStall || failed; }

  public void end(boolean interrupted) {
    // An interrupted/failed session must never create a valid height reference.
    boolean resetSucceeded = !interrupted && !failed && detectedStall && elevator.completeHoming();
    if (!interrupted && !failed && detectedStall && !resetSucceeded) {
      Logger.recordOutput("Commands/ElevatorHoming/FailureReason", "Height reset failed");
    }
    elevator.stop();
    timeout.stop();
    Logger.recordOutput("Commands/ElevatorHoming/ResetSucceeded", resetSucceeded);
    Logger.recordOutput("Commands/ElevatorHoming/CommandActive", false);
    Logger.recordOutput("Commands/ElevatorHoming/Interrupted", interrupted);
  }

  @Override
  public void onCancel(){
    end(true);
  }

  @Override
  public String name() {
    return "Elevator:Homing";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(elevator);
  }
}
