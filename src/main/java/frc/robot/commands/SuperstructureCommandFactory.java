// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.commands;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import java.util.Objects;
import java.util.OptionalInt;
import java.util.function.Predicate;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import frc.robot.constants.CoralHandlingConstants;
import static frc.robot.constants.CoralHandlingConstants.GRAB_ENTRY_HEIGHT_DROP;
import static frc.robot.constants.CoralHandlingConstants.INDEXER_PICKUP;
import static frc.robot.constants.CoralHandlingConstants.INDEXER_STORE;
import static frc.robot.constants.CoralHandlingConstants.PLACE_EXIT_HEIGHT_DROP;
import static frc.robot.constants.CoralHandlingConstants.REEF_HEADING_TOLERANCE_RADIANS;
import static frc.robot.constants.CoralHandlingConstants.REEF_POSITION_TOLERANCE_METERS;
import static frc.robot.constants.CoralHandlingConstants.STOW;
import static frc.robot.constants.CoralHandlingConstants.VERTICAL_UP_PIVOT_ANGLE;
import static frc.robot.constants.CoralHandlingConstants.reefHeight;
import static frc.robot.constants.CoralHandlingConstants.reefPlaceAngle;
import static frc.robot.constants.CoralHandlingConstants.reefPreparationAngle;
import frc.robot.commands.ReefAlignCommand.AlignmentTarget;
import frc.robot.constants.InventoryScanConstants;
import frc.robot.constants.ReefAlignConstants.RobotSide;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.coralsensor.CoralSensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ArmPose;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.spinindexer.CoralState;
import frc.robot.subsystems.spinindexer.SpinIndexer;
import frc.robot.subsystems.spinindexer.SpinIndexerConstants;
import frc.robot.util.ReefTargetColors.Level;

/**
 * Builds the action command bound to each state of the Superstructure's v3 state machine. The
 * machine in {@link Superstructure#controlCommand()} owns the states, names, and transitions; this
 * factory only constructs the coroutine sequences that drive the mechanisms and update the
 * Superstructure's coral records through its accessors.
 *
 * <p>Every builder is static and receives exactly the mechanisms it commands, so each signature
 * also documents the action's dependencies. Every command assumes it runs under a state command
 * that reserves the complete requirements of its action (see the state table in Superstructure).
 * Successful finite motion retains its last position loop while the parent waits for the next
 * step; a failed safe move requests a stop, halts the state's mechanisms, and lets the machine
 * fall into STOPPED. Configuration gates park the action — 2910-style — until an operator
 * transition cancels it.
 */
public final class SuperstructureCommandFactory {
  /** Static builders only; never instantiated. */
  private SuperstructureCommandFactory() {}

  /** Holds every mechanism still and parks; the machine stays here until a transition fires. */
  public static Command stoppedAction(Elevator elevator, SpinIndexer indexer) {
    return Command.noRequirements(coroutine -> {
      // drive.stop();
      elevator.stop();
      indexer.stop();
      coroutine.park();
    }).named("Superstructure:Stopped");
  }

  /** Homes the elevator; a failed homing requests a stop so the machine falls into STOPPED. */
  public static Command homingAction(Superstructure superstructure, Elevator elevator) {
    return Command.noRequirements(coroutine -> {
      coroutine.await(elevator.homingCommand());
      superstructure.setStatusStep(elevator.isPositionControlReady() ? "COMPLETE" : "FAILED");
      if (!elevator.isPositionControlReady()) superstructure.requestStop();
    }).named("Superstructure:Home");
  }

  /** Holds the stow pose with whatever the jaw currently has. */
  public static Command holdDefault(Superstructure superstructure, Elevator elevator) {
    return Command.noRequirements(coroutine -> {
      requireAutomation(superstructure, elevator, coroutine);
      if (!awaitSafeMove(superstructure, elevator, coroutine, STOW)) return;
      coroutine.park();
    }).named("Superstructure:HoldStow");
  }

  /** Holds the prepared reef pose for the last completed preparation. */
  public static Command holdPrepared(
      Superstructure superstructure, Elevator elevator, Level level) {
    return Command.noRequirements(coroutine -> {
      requireAutomation(superstructure, elevator, coroutine);
      if (superstructure.getSelectedSide() == null) {
        block(superstructure, elevator, coroutine, "NO_PREPARED_SIDE");
      }
      ArmPose pose =
          preparation(level, superstructure.getEndEffectorCoralState(), superstructure.getSelectedSide());
      if (!awaitSafeMove(superstructure, elevator, coroutine, pose)) return;
      coroutine.park();
    }).named("Superstructure:HoldPrepared");
  }

  /** Returns the arm to stow and forgets any reef preparation. */
  public static Command stowArm(
      Superstructure superstructure, Elevator elevator, SpinIndexer indexer) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(superstructure, elevator, coroutine);
      superstructure.clearPreparation();
      if (!awaitSafeMove(superstructure, elevator, coroutine, STOW)) return;
      completeAction(superstructure);
    }).named("Superstructure:StowArm");
  }

  /** Moves to the reef preparation pose for the level and records the selection on arrival. */
  public static Command prepareArm(
      Superstructure superstructure,
      Drive drive,
      Elevator elevator,
      SpinIndexer indexer,
      Level level) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(superstructure, elevator, coroutine);
      // Invalidate any stale selection; alignment stays owned by the bumpers.
      superstructure.setPreparation(null, null);
      if (superstructure.getEndEffectorCoralState() == CoralState.UNKNOWN) {
        block(superstructure, elevator, coroutine, "UNKNOWN_GRIPPER");
      }
      var selection = ReefAlignCommand.selectReef(drive.getPose());
      if (selection.isEmpty()) block(superstructure, elevator, coroutine, "NO_REEF_SIDE");
      RobotSide side = selection.orElseThrow().side();
      if (superstructure.getEndEffectorCoralState() == CoralState.EMPTY
          && !CoralHandlingConstants.positive(GRAB_ENTRY_HEIGHT_DROP.in(Meters))) {
        block(superstructure, elevator, coroutine, "INVALID_GRAB_OFFSET");
      }
      ArmPose pose = preparation(level, superstructure.getEndEffectorCoralState(), side);
      if (!awaitSafeMove(superstructure, elevator, coroutine, pose)) return;
      superstructure.setPreparation(level, side);
      completeAction(superstructure);
    }).named("Superstructure:PrepareArm");
  }

  /** Raise clear, align one fixed slot, descend open, detect, clamp, and extract. */
  public static Command getCoral(
      Superstructure superstructure,
      Elevator elevator,
      SpinIndexer indexer,
      EndEffector jaw,
      CoralSensor coralSensor,
      CoralState wanted) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(superstructure, elevator, coroutine);
      if (superstructure.getEndEffectorCoralState() != CoralState.EMPTY) {
        block(superstructure, elevator, coroutine, "GRIPPER_NOT_EMPTY");
      }
      OptionalInt matchingSlot = findNearestSlot(superstructure, indexer, wanted);
      if (matchingSlot.isEmpty()) block(superstructure, elevator, coroutine, "NO_MATCHING_SLOT");
      int slot = matchingSlot.orElseThrow();
      superstructure.clearPreparation();

      superstructure.setStatusStep("CLEARING_INDEXER");
      if (!awaitSafeMove(superstructure, elevator, coroutine, STOW)) return;
      // Empty jaw opening and turntable alignment may overlap only after the arm is clear.
      var opening = coroutine.fork(jaw.openCommand());
      coroutine.await(indexer.rotateToSlotCommand(slot));
      opening.awaitCompletion();
      if (!awaitSafeMove(superstructure, elevator, coroutine, INDEXER_PICKUP)) return;
      superstructure.setStatusStep("WAITING_FOR_CORAL");
      observeEndEffector(superstructure, elevator, coralSensor, coroutine,
          SuperstructureCommandFactory::isCoral);
      coroutine.waitUntil(() -> elevator.hasReached(INDEXER_PICKUP)
          && indexer.isSlotSettled(slot)
          && coralSensor.getEndEffectorObservation()
              .filter(SuperstructureCommandFactory::isCoral)
              .isPresent());

      // From the actuation request until post-retreat confirmation, neither record is certain.
      superstructure.setSlotCoralState(slot, CoralState.UNKNOWN);
      superstructure.setEndEffectorCoralState(CoralState.UNKNOWN);
      coroutine.await(jaw.clampCommand());
      if (!awaitSafeMove(superstructure, elevator, coroutine, STOW)) return;
      superstructure.setStatusStep("CONFIRMING_PICKUP");
      CoralState grabbed = observeEndEffector(superstructure, elevator, coralSensor, coroutine,
          SuperstructureCommandFactory::isCoral);
      superstructure.setEndEffectorCoralState(grabbed);
      superstructure.setSlotCoralState(slot, CoralState.EMPTY);
      completeAction(superstructure);
    }).named("Superstructure:GetCoral");
  }

  /** Stores the held coral into the nearest empty slot and confirms the release. */
  public static Command storeCoral(
      Superstructure superstructure,
      Elevator elevator,
      SpinIndexer indexer,
      EndEffector jaw,
      CoralSensor coralSensor) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(superstructure, elevator, coroutine);
      CoralState held = superstructure.getEndEffectorCoralState();
      if (!isCoral(held)) block(superstructure, elevator, coroutine, "NO_CONFIRMED_CORAL");
      OptionalInt emptySlot = findNearestSlot(superstructure, indexer, CoralState.EMPTY);
      if (emptySlot.isEmpty()) block(superstructure, elevator, coroutine, "NO_EMPTY_SLOT");
      int slot = emptySlot.orElseThrow();
      superstructure.clearPreparation();

      superstructure.setStatusStep("CLEARING_INDEXER");
      if (!awaitSafeMove(superstructure, elevator, coroutine, STOW)) return;
      coroutine.await(indexer.rotateToSlotCommand(slot));
      if (!awaitSafeMove(superstructure, elevator, coroutine, INDEXER_STORE)) return;
      coroutine.waitUntil(() -> elevator.hasReached(INDEXER_STORE) && indexer.isSlotSettled(slot));
      superstructure.setSlotCoralState(slot, CoralState.UNKNOWN);
      superstructure.setEndEffectorCoralState(CoralState.UNKNOWN);
      superstructure.setStatusStep("RELEASING");
      coroutine.await(jaw.openCommand());
      if (!awaitSafeMove(superstructure, elevator, coroutine, STOW)) return;
      superstructure.setStatusStep("CONFIRMING_RELEASE");
      observeEndEffector(superstructure, elevator, coralSensor, coroutine,
          state -> state == CoralState.EMPTY);
      superstructure.setSlotCoralState(slot, held);
      superstructure.setEndEffectorCoralState(CoralState.EMPTY);
      completeAction(superstructure);
    }).named("Superstructure:StoreCoral");
  }

  /** Scan progress belongs to this invocation; interrupted scans never commit partial inventory. */
  public static Command scanSlots(
      Superstructure superstructure, Elevator elevator, SpinIndexer indexer, CoralSensor coralSensor) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(superstructure, elevator, coroutine);
      superstructure.clearPreparation();
      if (!InventoryScanConstants.turntableConfigured()) {
        block(superstructure, elevator, coroutine, "SCAN_UNCONFIGURED");
      }
      ArmPose clearance = new ArmPose(
          Meters.of(Math.max(elevator.getHeight().in(Meters),
              CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
                  + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters)
                  + ElevatorConstants.POSITION_TOLERANCE.in(Meters))),
          elevator.getPivotAngle());
      if (!awaitSafeMove(superstructure, elevator, coroutine, clearance)) return;
      CoralState[] scanned = new CoralState[SpinIndexerConstants.SLOT_COUNT];
      for (int slot = 0; slot < scanned.length; slot++) {
        superstructure.setStatusStep("SLOT_" + (slot + 1));
        Angle target = InventoryScanConstants.slotAngle(slot);
        coroutine.await(indexer.rotateToAngleCommand(target,
            InventoryScanConstants.ALIGNMENT_TOLERANCE, InventoryScanConstants.SETTLE_SECONDS));
        waitForSensorFlush(superstructure, elevator, coroutine);
        coralSensor.resetIndexerObservation();
        coroutine.waitUntil(() -> coralSensor.getIndexerObservation().isPresent());
        scanned[slot] = coralSensor.getIndexerObservation().orElseThrow();
      }
      for (int slot = 0; slot < scanned.length; slot++) {
        superstructure.setSlotCoralState(slot, scanned[slot]);
      }
      completeAction(superstructure);
    }).named("Superstructure:ScanSlots");
  }

  /** Grabs a coral from the aligned reef branch at the prepared level. */
  public static Command pickUpCoral(
      Superstructure superstructure,
      Drive drive,
      Elevator elevator,
      EndEffector jaw,
      CoralSensor coralSensor,
      Level level) {
    return Command.noRequirements(coroutine -> {
      drive.stop();
      requireAutomation(superstructure, elevator, coroutine);
      if (superstructure.getEndEffectorCoralState() != CoralState.EMPTY) {
        block(superstructure, elevator, coroutine, "GRIPPER_NOT_EMPTY");
      }
      AlignmentTarget target = superstructure.getAlignment();
      if (target == null || superstructure.getSelectedSide() != target.side()) {
        block(superstructure, elevator, coroutine, "NO_MATCHING_ALIGNMENT");
      }
      ArmPose entry = preparation(level, CoralState.EMPTY, target.side());
      ArmPose atBranch = grabPose(level, target.side());
      ArmPose away = new ArmPose(atBranch.height(), reefPreparationAngle(target.side()));
      if (!CoralHandlingConstants.positive(GRAB_ENTRY_HEIGHT_DROP.in(Meters))
          || !entry.configured() || !atBranch.configured() || !away.configured()) {
        block(superstructure, elevator, coroutine, "INVALID_REEF_POSES");
      }

      if (!awaitSafeMove(superstructure, elevator, coroutine, entry)) return;
      coroutine.await(jaw.openCommand());
      superstructure.setStatusStep("WAITING_FOR_ALIGNMENT");
      coroutine.waitUntil(() -> reefPoseReached(superstructure, drive, target));
      coroutine.await(elevator.moveToCommand(atBranch));
      superstructure.setStatusStep("WAITING_FOR_CORAL");
      observeEndEffector(superstructure, elevator, coralSensor, coroutine,
          SuperstructureCommandFactory::isCoral);
      coroutine.waitUntil(() -> reefPoseReached(superstructure, drive, target)
          && elevator.hasReached(atBranch)
          && coralSensor.getEndEffectorObservation()
              .filter(SuperstructureCommandFactory::isCoral)
              .isPresent());
      superstructure.setEndEffectorCoralState(CoralState.UNKNOWN);
      superstructure.setStatusStep("CLAMPING");
      coroutine.await(jaw.clampCommand());
      if (!awaitSafeMove(superstructure, elevator, coroutine, away)) return;
      superstructure.setStatusStep("CONFIRMING_PICKUP");
      superstructure.setEndEffectorCoralState(
          observeEndEffector(superstructure, elevator, coralSensor, coroutine,
              SuperstructureCommandFactory::isCoral));
      completeAction(superstructure);
    }).named("Superstructure:PickUpCoral");
  }

  /** Places the held coral onto the aligned reef branch at the prepared level. */
  public static Command placeCoral(
      Superstructure superstructure,
      Drive drive,
      Elevator elevator,
      EndEffector jaw,
      CoralSensor coralSensor,
      Level level) {
    return Command.noRequirements(coroutine -> {
      drive.stop();
      requireAutomation(superstructure, elevator, coroutine);
      if (!isCoral(superstructure.getEndEffectorCoralState())) {
        block(superstructure, elevator, coroutine, "NO_CONFIRMED_CORAL");
      }
      AlignmentTarget target = superstructure.getAlignment();
      if (target == null || superstructure.getSelectedSide() != target.side()) {
        block(superstructure, elevator, coroutine, "NO_MATCHING_ALIGNMENT");
      }
      ArmPose atBranch = new ArmPose(reefHeight(level), reefPlaceAngle(target.side()));
      Distance exitHeight =
          Meters.of(atBranch.height().in(Meters) - PLACE_EXIT_HEIGHT_DROP.in(Meters));
      ArmPose clear = new ArmPose(exitHeight, VERTICAL_UP_PIVOT_ANGLE);
      if (!CoralHandlingConstants.positive(PLACE_EXIT_HEIGHT_DROP.in(Meters))
          || !atBranch.configured() || !clear.configured()
          || clear.height().in(Meters) < CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
              + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters)
              + ElevatorConstants.POSITION_TOLERANCE.in(Meters)) {
        block(superstructure, elevator, coroutine, "INVALID_REEF_POSES");
      }

      if (!awaitSafeMove(superstructure, elevator, coroutine, atBranch)) return;
      superstructure.setStatusStep("WAITING_FOR_ALIGNMENT");
      coroutine.waitUntil(() -> reefPoseReached(superstructure, drive, target)
          && elevator.hasReached(atBranch));
      superstructure.setEndEffectorCoralState(CoralState.UNKNOWN);
      superstructure.setStatusStep("RELEASING");
      coroutine.await(jaw.openCommand());
      // This calibrated Reef exit intentionally lowers and pivots concurrently.
      coroutine.await(elevator.moveToCommand(clear));
      superstructure.setStatusStep("CONFIRMING_RELEASE");
      observeEndEffector(superstructure, elevator, coralSensor, coroutine,
          state -> state == CoralState.EMPTY);
      superstructure.setEndEffectorCoralState(CoralState.EMPTY);
      completeAction(superstructure);
    }).named("Superstructure:PlaceCoral");
  }

  // ========== Action plumbing ==========
  // These helpers only run under a state command, which reserves their complete requirements.

  /** Moves through clearance and reports failure; a failed move aborts the enclosing action. */
  private static boolean awaitSafeMove(
      Superstructure superstructure, Elevator elevator, Coroutine coroutine, ArmPose pose) {
    ElevatorMoveSafelyCommand movement = new ElevatorMoveSafelyCommand(elevator, pose);
    coroutine.await(movement);
    if (movement.hasSucceeded()) return true;
    // await reports termination, so failed travel must explicitly abort the enclosing action.
    superstructure.setStatusStep("ELEVATOR_MOVE_FAILED");
    superstructure.requestStop();
    superstructure.stopOwnedMechanisms();
    return false;
  }

  private static void requireAutomation(
      Superstructure superstructure, Elevator elevator, Coroutine coroutine) {
    if (!CoralHandlingConstants.AUTOMATION_CONFIGURED
        || !CoralHandlingConstants.clearanceConfigured()) {
      block(superstructure, elevator, coroutine, "AUTOMATION_UNCONFIGURED");
    }
  }

  /** Holds still and parks the action until an operator transition cancels its state. */
  private static void block(
      Superstructure superstructure, Elevator elevator, Coroutine coroutine, String reason) {
    elevator.holdStill();
    superstructure.setStatusStep(reason);
    coroutine.park();
  }

  private static void completeAction(Superstructure superstructure) {
    superstructure.setStatusStep("COMPLETE");
  }

  private static void waitForSensorFlush(
      Superstructure superstructure, Elevator elevator, Coroutine coroutine) {
    if (!CoralHandlingConstants.positive(InventoryScanConstants.SENSOR_FLUSH_SECONDS)) {
      block(superstructure, elevator, coroutine, "SENSOR_FLUSH_UNCONFIGURED");
    }
    coroutine.wait(Seconds.of(InventoryScanConstants.SENSOR_FLUSH_SECONDS));
  }

  /** Discard in-motion/integration samples and require a new stable reading at this pose. */
  private static CoralState observeEndEffector(
      Superstructure superstructure,
      Elevator elevator,
      CoralSensor coralSensor,
      Coroutine coroutine,
      Predicate<CoralState> accepted) {
    waitForSensorFlush(superstructure, elevator, coroutine);
    coralSensor.resetEndEffectorObservation();
    coroutine.waitUntil(
        () -> coralSensor.getEndEffectorObservation().filter(accepted).isPresent());
    return coralSensor.getEndEffectorObservation().orElseThrow();
  }

  private static boolean isCoral(CoralState state) {
    return state == CoralState.BLACK || state == CoralState.WHITE;
  }

  /**
   * True when the chassis sits on the last aligned reef pose within the configured tolerances —
   * 2910's isReadyToEject idea: the chassis must be on the branch before grabbing from it or
   * releasing onto it.
   */
  private static boolean reefPoseReached(
      Superstructure superstructure, Drive drive, AlignmentTarget target) {
    return target != null
        && target.equals(superstructure.getAlignment())
        && drive.isWithinPose(
            target.pose(), REEF_POSITION_TOLERANCE_METERS, REEF_HEADING_TOLERANCE_RADIANS);
  }

  /** Finds the shortest rotation to slots with matching contents; EMPTY can select storage. */
  private static OptionalInt findNearestSlot(
      Superstructure superstructure, SpinIndexer indexer, CoralState state) {
    Objects.requireNonNull(state, "state");
    if (!indexer.isPositionControlReady()) {
      return OptionalInt.empty();
    }
    int nearestSlot = -1;
    double shortestDistance = Double.POSITIVE_INFINITY;
    for (int slot = 0; slot < SpinIndexerConstants.SLOT_COUNT; slot++) {
      if (superstructure.getSlotCoralState(slot) != state) {
        continue;
      }
      double distance = Math.abs(indexer.angleToSlot(slot));
      // Ascending slot order resolves equal-distance choices deterministically.
      if (distance < shortestDistance - 1.0e-9) {
        shortestDistance = distance;
        nearestSlot = slot;
      }
    }
    return nearestSlot == -1 ? OptionalInt.empty() : OptionalInt.of(nearestSlot);
  }

  /** The parked pose for the jaw's contents; an empty jaw parks a grab-entry drop below grabbing. */
  private static ArmPose preparation(Level level, CoralState held, RobotSide side) {
    Distance height = reefHeight(level);
    if (held == CoralState.EMPTY) {
      return new ArmPose(
          Meters.of(height.in(Meters) - GRAB_ENTRY_HEIGHT_DROP.in(Meters)), reefPlaceAngle(side));
    }
    return new ArmPose(height, reefPreparationAngle(side));
  }

  /** The true grab pose where LT clamps: the parked empty height risen by the grab entry drop. */
  private static ArmPose grabPose(Level level, RobotSide side) {
    return new ArmPose(reefHeight(level), reefPlaceAngle(side));
  }
}
