// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.commands;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Objects;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import frc.robot.constants.CoralHandlingConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ArmPose;

/** Raises, swings, then lowers; success retains the position loop, failure or cancellation stops. */
public final class ElevatorMoveSafelyCommand implements Command {
  private enum Phase {
    RAISING,
    SWINGING,
    FINAL_POSE
  }

  private final Elevator elevator;
  private final ArmPose target;
  private ArmPose swingPose;
  private ArmPose phaseTarget;
  private Phase phase;
  private boolean reachedTarget;
  private boolean failed;

  public ElevatorMoveSafelyCommand(Elevator elevator, ArmPose target) {
    this.elevator = Objects.requireNonNull(elevator, "elevator");
    this.target = Objects.requireNonNull(target, "target");
  }

  @Override
  public void run(Coroutine c) {
    reachedTarget = false;
    failed = false;
    swingPose = null;
    phaseTarget = null;
    phase = null;
    Logger.recordOutput("Commands/ElevatorMoveSafely/Active", true);
    Logger.recordOutput("Commands/ElevatorMoveSafely/Completed", false);
    Logger.recordOutput("Commands/ElevatorMoveSafely/Interrupted", false);
    Logger.recordOutput("Commands/ElevatorMoveSafely/FailureReason", "");
    if (!target.configured() || !CoralHandlingConstants.clearanceConfigured()
        || !elevator.isPositionControlReady()) {
      fail("UNAVAILABLE");
    } else {
      var initialPivot = elevator.getPivotAngle();
      if (Math.abs(target.pivot().in(Radians) - initialPivot.in(Radians))
          > ElevatorConstants.PIVOT_TOLERANCE.in(Radians)) {
        // Keep clearance until the measured pivot has settled, not just until its target is set.
        double clearance = CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
            + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters)
            + ElevatorConstants.POSITION_TOLERANCE.in(Meters);
        var travelHeight = Meters.of(Math.max(clearance,
            Math.max(elevator.getHeight().in(Meters), target.height().in(Meters))));
        ArmPose raisedPose = new ArmPose(travelHeight, initialPivot);
        swingPose = new ArmPose(travelHeight, target.pivot());
          commandPose(raisedPose, Phase.RAISING);
        
      } else {
        commandPose(target, Phase.FINAL_POSE);
      }
    }

    while (!isFinished()) {
      if (!elevator.isPositionControlReady()) {
        fail("FEEDBACK_UNAVAILABLE");
      } else if (elevator.hasReached(phaseTarget)) {
        switch (phase) {
          case RAISING -> commandPose(swingPose, Phase.SWINGING);
          case SWINGING -> commandPose(target, Phase.FINAL_POSE);
          case FINAL_POSE -> reachedTarget = true;
        }
      }
      if (!isFinished()) c.yield();
    }
    end(false);
  }

  private void commandPose(ArmPose pose, Phase nextPhase) {
    phaseTarget = pose;
    phase = nextPhase;
    Logger.recordOutput("Commands/ElevatorMoveSafely/Phase", phase.name());
    elevator.setTargets(pose);
  }

  private void fail(String reason) {
    failed = true;
    elevator.stop();
    Logger.recordOutput("Commands/ElevatorMoveSafely/Phase", reason);
    Logger.recordOutput("Commands/ElevatorMoveSafely/FailureReason", reason);
  }

  public boolean isFinished() {
    return reachedTarget || failed;
  }

  /** An awaiting parent must check the outcome before starting another mechanism action. */
  public boolean hasSucceeded() {
    return reachedTarget && !failed;
  }

  public void end(boolean interrupted) {
    // Normal arrival retains the position loop while the parent awaits its next mechanism.
    if (interrupted || failed) elevator.stop();
    boolean completed = !interrupted && !failed && reachedTarget;
    if (completed) Logger.recordOutput("Commands/ElevatorMoveSafely/Phase", "COMPLETE");
    Logger.recordOutput("Commands/ElevatorMoveSafely/Completed", completed);
    Logger.recordOutput("Commands/ElevatorMoveSafely/Interrupted", interrupted);
  }

  @Override
  public void onCancel() {
    end(true);
  }

  @Override
  public void onExit() {
    Logger.recordOutput("Commands/ElevatorMoveSafely/Active", false);
  }

  @Override
  public String name() {
    return "Elevator:MoveSafely";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(elevator);
  }
}
