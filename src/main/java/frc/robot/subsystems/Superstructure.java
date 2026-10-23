// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Objects;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.Predicate;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Pose2d;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.command3.Trigger;
import frc.robot.commands.ElevatorMoveSafelyCommand;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.commands.ReefAlignCommand.AlignmentTarget;
import frc.robot.constants.CoralHandlingConstants;
import static frc.robot.constants.CoralHandlingConstants.INDEXER_PICKUP;
import static frc.robot.constants.CoralHandlingConstants.INDEXER_STORE;
import static frc.robot.constants.CoralHandlingConstants.GRAB_ENTRY_HEIGHT_DROP;
import static frc.robot.constants.CoralHandlingConstants.PLACE_EXIT_HEIGHT_DROP;
import static frc.robot.constants.CoralHandlingConstants.VERTICAL_UP_PIVOT_ANGLE;
import static frc.robot.constants.CoralHandlingConstants.REEF_HEADING_TOLERANCE_RADIANS;
import static frc.robot.constants.CoralHandlingConstants.REEF_POSITION_TOLERANCE_METERS;
import static frc.robot.constants.CoralHandlingConstants.STOW;
import static frc.robot.constants.CoralHandlingConstants.reefHeight;
import static frc.robot.constants.CoralHandlingConstants.reefPlaceAngle;
import static frc.robot.constants.CoralHandlingConstants.reefPreparationAngle;
import frc.robot.constants.InventoryScanConstants;
import frc.robot.constants.ReefAlignConstants.RobotSide;
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
 * Coral superstructure whose {@link #controlCommand()} builds a Commands v3 {@link StateMachine}.
 * Each invocation owns its state nodes; {@link SuperState} only identifies states for telemetry.
 * The gamepad lives here as well: its button triggers define
 * direct {@code switchTo(...).when(...)} edges between machine states, with no parallel requested
 * state. Action states complete naturally and use {@code whenComplete()} to return to the
 * inventory-appropriate ready/held/prepared state. Each action is a coroutine sequence whose
 * child commands wait for motion, jaw actuation, and turntable settling.
 *
 * <p>This class also owns the robot's global coral records — slot1CoralState, slot2CoralState,
 * slot3CoralState and endEffectorCoralState, plus the operator context selectedLevel,
 * selectedSide and alignment. Actions update only these records; no other copy of the coral
 * state exists anywhere in the code. Every record is logged every cycle, and an action that
 * finishes updates the records to reflect the confirmed observations.
 *
 * <p>Gates hold an action at its current step until conditions are met, 2910-style: the jaw must
 * be free before taking a coral and holding one before storing or placing it, and the chassis
 * must sit on the aligned reef pose — with the arm at the branch — before grabbing from or
 * releasing onto it. State commands reserve all mechanisms needed by the complete action,
 * including sensor waits. Normal motion completion retains closed-loop holding; cancellation
 * stops owned motors and leaves the jaw valve unchanged.
 */
public final class Superstructure {
  public enum SuperState {
    STOPPED,
    HOMING,
    READY_EMPTY,
    READY_HOLDING_CORAL,
    SCANNING_CORAL,
    PREPARED_L2,
    PREPARED_L3,
    PREPARED_L4,
    STOWING,
    STORING_CORAL,
    GETTING_BLACK_CORAL,
    GETTING_WHITE_CORAL,
    PREPARING_L2,
    PREPARING_L3,
    PREPARING_L4,
    PICKING_UP_L2,
    PICKING_UP_L3,
    PICKING_UP_L4,
    PLACING_L2,
    PLACING_L3,
    PLACING_L4
  }

  private final Drive drive;
  private final Elevator elevator;
  private final SpinIndexer indexer;
  private final EndEffector jaw;
  private final CoralSensor coralSensor;
  private final CommandXboxController gamepad = new CommandXboxController(0);
  private final Trigger scanButton = gamepad.dpadUp().risingEdge();
  private final Trigger getBlackButton = gamepad.dpadLeft().risingEdge();
  private final Trigger getWhiteButton = gamepad.dpadRight().risingEdge();
  private final Trigger stowOrStoreButton = gamepad.a().risingEdge();
  private final Trigger prepareL2Button = gamepad.b().risingEdge();
  private final Trigger prepareL3Button = gamepad.x().risingEdge();
  private final Trigger prepareL4Button = gamepad.y().risingEdge();
  private final Trigger pickupButton =
      gamepad.leftTrigger(CoralHandlingConstants.TRIGGER_THRESHOLD).risingEdge();
  private final Trigger placeButton =
      gamepad.rightTrigger(CoralHandlingConstants.TRIGGER_THRESHOLD).risingEdge();
  private final Trigger homingButton = gamepad.view().risingEdge();

  private SuperState currentSuperState = SuperState.STOPPED;
  private SuperState previousSuperState = SuperState.STOPPED;
  private boolean stopRequested;
  private String status = "PRELOAD_REQUIRED";

  // ========== The robot's global records ==========
  // The only copies of the coral and operator state anywhere in the code. Actions write them;
  // everything else reads them. All of them are logged every cycle.

  private Level selectedLevel; // level of the last completed preparation
  private RobotSide selectedSide; // reef side of the last completed preparation
  private AlignmentTarget alignment; // reef face targeted by the bumpers
  private CoralState slot1CoralState = CoralState.EMPTY;
  private CoralState slot2CoralState = CoralState.EMPTY;
  private CoralState slot3CoralState = CoralState.EMPTY;
  private CoralState endEffectorCoralState = CoralState.EMPTY;

  public Superstructure(
      Drive drive, Elevator elevator, SpinIndexer indexer, EndEffector jaw, CoralSensor coralSensor) {
    this.drive = drive;
    this.elevator = elevator;
    this.indexer = indexer;
    this.jaw = jaw;
    this.coralSensor = coralSensor;
  }

  /** Builds a fresh teleop state graph; the caller owns scheduling and cancellation. */
  public Command controlCommand() {
    StateMachine stateMachine = new StateMachine("Superstructure");
    EnumMap<SuperState, State> states = new EnumMap<>(SuperState.class);
    for (SuperState state : SuperState.values()) {
      states.put(state, stateMachine.addState(stateCommand(state)));
    }
    stateMachine.setInitialState(states.get(defaultCurrentState()));
    for (SuperState fromState : SuperState.values()) {
      State fromStateNode = states.get(fromState);
      fromStateNode.switchTo(states.get(SuperState.STOPPED)).when(() -> stopRequested);
      if (fromState != SuperState.HOMING) {
        addGamepadTransitions(states, fromState);
      }
      if (isReadyState(fromState) || fromState == SuperState.STOPPED) {
        fromStateNode.switchTo(states.get(SuperState.HOMING)).when(homingButton);
      }
      if (isReadyState(fromState)) {
        fromStateNode.switchTo(() -> states.get(defaultCurrentState()))
            .when(() -> defaultCurrentState() != currentSuperState);
      }
      if (fromState != SuperState.STOPPED) {
        fromStateNode.switchTo(() -> states.get(
            stopRequested ? SuperState.STOPPED : defaultCurrentState())).whenComplete();
      }
    }
    return Command.noRequirements(coroutine -> coroutine.await(stateMachine))
        .whenExited(this::clearRuntimeState)
        .named("Superstructure:Teleop");
  }

  private void addGamepadTransitions(EnumMap<SuperState, State> states, SuperState fromState) {
    State fromStateNode = states.get(fromState);
    transition(states, fromState, SuperState.SCANNING_CORAL, scanButton);
    transition(states, fromState, SuperState.GETTING_BLACK_CORAL, getBlackButton);
    transition(states, fromState, SuperState.GETTING_WHITE_CORAL, getWhiteButton);
    fromStateNode.switchTo(() -> states.get(stowOrStoreState()))
        .when(() -> stowOrStoreState() != fromState && stowOrStoreButton.getAsBoolean());
    transition(states, fromState, SuperState.PREPARING_L2, prepareL2Button);
    transition(states, fromState, SuperState.PREPARING_L3, prepareL3Button);
    transition(states, fromState, SuperState.PREPARING_L4, prepareL4Button);
    fromStateNode.switchTo(() -> states.get(pickupStateForLevel(selectedLevel)))
      .when(() -> pickupStateForLevel(selectedLevel) != null
          && pickupStateForLevel(selectedLevel) != fromState
          && pickupButton.getAsBoolean());
    fromStateNode.switchTo(() -> states.get(placeStateForLevel(selectedLevel)))
      .when(() -> placeStateForLevel(selectedLevel) != null
          && placeStateForLevel(selectedLevel) != fromState
          && placeButton.getAsBoolean());
  }

  private void transition(
      EnumMap<SuperState, State> states,
      SuperState fromState, SuperState commandedState, Trigger button) {
    if (fromState != commandedState) {
      states.get(fromState).switchTo(states.get(commandedState)).when(button);
    }
  }

  private SuperState stowOrStoreState() {
    return endEffectorCoralState == CoralState.EMPTY
        ? SuperState.STOWING : SuperState.STORING_CORAL;
  }

  private static boolean isReadyState(SuperState state) {
    return state == SuperState.READY_EMPTY
        || state == SuperState.READY_HOLDING_CORAL
        || state == SuperState.PREPARED_L2
        || state == SuperState.PREPARED_L3
        || state == SuperState.PREPARED_L4;
  }

  public CommandXboxController gamepad() {
    return gamepad;
  }

  // ========== Record access ==========
  // Actions write the records directly; other classes only read through these.

  /** Slot indices are zero-based: 0, 1, 2 correspond to dashboard Slot1, Slot2, Slot3. */
  public CoralState getSlotCoralState(int slot) {
    return switch (slot) {
      case 0 -> slot1CoralState;
      case 1 -> slot2CoralState;
      case 2 -> slot3CoralState;
      default -> throw new IndexOutOfBoundsException("Coral slot must be 0, 1, or 2: " + slot);
    };
  }

  public CoralState getEndEffectorCoralState() {
    return endEffectorCoralState;
  }

  private void setSlotCoralState(int slot, CoralState state) {
    Objects.requireNonNull(state, "state");
    switch (slot) {
      case 0 -> slot1CoralState = state;
      case 1 -> slot2CoralState = state;
      case 2 -> slot3CoralState = state;
      default -> throw new IndexOutOfBoundsException("Coral slot must be 0, 1, or 2: " + slot);
    }
  }

  /** Applies the four preload choosers and forgets any stale operator choices. */
  public void applyPreload(
      CoralState slot1, CoralState slot2, CoralState slot3, CoralState gripperCoral) {
    slot1CoralState = Objects.requireNonNull(slot1, "slot1");
    slot2CoralState = Objects.requireNonNull(slot2, "slot2");
    slot3CoralState = Objects.requireNonNull(slot3, "slot3");
    endEffectorCoralState = Objects.requireNonNull(gripperCoral, "gripperCoral");
    selectedLevel = null;
    selectedSide = null;
    alignment = null;
    // On the next state completion, the dynamic completion edge selects from the updated inventory.
    status = "PRELOAD_APPLIED";
  }

  public boolean isBusy() {
    return isActionState(currentSuperState);
  }

  public boolean isDriveLocked() {
    return isReefAction(currentSuperState);
  }

  public void observeAlignment(AlignmentTarget target) {
    alignment = target;
  }

  /** True when the elevator is homed and calibrated, so position commands are meaningful. */
  public boolean isElevatorReady() {
    return elevator.isPositionControlReady();
  }

  /**
   * Stops every mechanism immediately and holds the STOPPED superstate — 2910's stopped() —
   * until another wanted state preempts it.
   */
  public void stop() {
    // drive.stop();
    elevator.stop();
    indexer.stop();
    stopRequested = true;
  }

  /**
   * The mechanisms an auto sequence must own while it runs, as opaque scheduling handles, so
   * nothing else — like a homing command — can fight the sequence mid-run.
   */
  public Set<Mechanism> autoRequirements() {
    return Set.of(drive, elevator, indexer, jaw);
  }

  // ========== Telemetry ==========

  public void periodic() {
    Logger.recordOutput("Superstructure/Busy", isBusy());
    Logger.recordOutput("Superstructure/DriveLocked", isDriveLocked());
    Logger.recordOutput("Superstructure/Status", status);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.name());
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState.name());
    Logger.recordOutput(
        "Superstructure/SelectedLevel", selectedLevel == null ? "UNSELECTED" : selectedLevel.name());
    Logger.recordOutput(
        "Superstructure/SelectedSide", selectedSide == null ? "UNSELECTED" : selectedSide.name());
    Logger.recordOutput(
        "Superstructure/Alignment",
        alignment == null ? new Pose2d[0] : new Pose2d[] {alignment.pose()});
    Logger.recordOutput(
        "Superstructure/ReefBranch",
        alignment == null ? "UNSELECTED" : alignment.branch().name());
    Logger.recordOutput(
        "Superstructure/AlignedRobotSide",
        alignment == null ? "UNSELECTED" : alignment.side().name());
    Logger.recordOutput("Superstructure/Slot1CoralState", slot1CoralState.name());
    Logger.recordOutput("Superstructure/Slot2CoralState", slot2CoralState.name());
    Logger.recordOutput("Superstructure/Slot3CoralState", slot3CoralState.name());
    Logger.recordOutput("Superstructure/EndEffectorCoralState", endEffectorCoralState.name());
  }

  /** Holding still, ready to work with whatever the jaw currently has. */
  private SuperState defaultCurrentState() {
    if (endEffectorCoralState == CoralState.UNKNOWN) return SuperState.STOPPED;
    if (selectedLevel != null && selectedSide != null) {
      return switch (selectedLevel) {
        case L2 -> SuperState.PREPARED_L2;
        case L3 -> SuperState.PREPARED_L3;
        case L4 -> SuperState.PREPARED_L4;
        default -> readyState();
      };
    }
    return readyState();
  }

  private SuperState readyState() {
    return endEffectorCoralState == CoralState.EMPTY
        ? SuperState.READY_EMPTY
        : SuperState.READY_HOLDING_CORAL;
  }

  private void enterState(SuperState state) {
    if (currentSuperState != state) {
      previousSuperState = currentSuperState;
      currentSuperState = state;
    }
    if (state == SuperState.STOPPED) {
      stopRequested = false;
    }
    if (state == SuperState.HOMING) {
      // Homing must not finish by returning to a previously prepared reef pose.
      selectedLevel = null;
      selectedSide = null;
      alignment = null;
    }
    if (isActionState(state)) {
      status = state.name();
    }
  }

  private void clearRuntimeState() {
    previousSuperState = currentSuperState;
    currentSuperState = SuperState.STOPPED;
    stopRequested = false;
    selectedLevel = null;
    selectedSide = null;
    alignment = null;
    status = "INACTIVE";
  }

  private Command stateCommand(SuperState state) {
    Set<Mechanism> requirements = requirementsFor(state);
    return Command.requiring(requirements)
        .executing(coroutine -> {
          // State.onEnter runs after the command's first execution, so initialize here.
          enterState(state);
          coroutine.await(actionCommand(state));
        })
        .whenCanceled(() -> stopMechanisms(requirements))
        .named("Superstructure:" + state.name());
  }

  private Set<Mechanism> requirementsFor(SuperState state) {
    return switch (state) {
      case STOPPED -> Set.of(elevator, indexer);
      case HOMING, READY_EMPTY, READY_HOLDING_CORAL, PREPARED_L2, PREPARED_L3, PREPARED_L4 ->
          Set.of(elevator);
      case STOWING, PREPARING_L2, PREPARING_L3, PREPARING_L4 -> Set.of(elevator, indexer);
      case SCANNING_CORAL -> Set.of(elevator, indexer, coralSensor);
      case STORING_CORAL, GETTING_BLACK_CORAL, GETTING_WHITE_CORAL ->
          Set.of(elevator, indexer, jaw, coralSensor);
      case PICKING_UP_L2, PICKING_UP_L3, PICKING_UP_L4,
          PLACING_L2, PLACING_L3, PLACING_L4 -> Set.of(drive, elevator, jaw, coralSensor);
    };
  }

  private Command actionCommand(SuperState state) {
    return switch (state) {
      case STOPPED -> Command.noRequirements(coroutine -> {
        // drive.stop();
        elevator.stop();
        indexer.stop();
        coroutine.park();
      }).named("Superstructure:Stopped");
      case HOMING -> Command.noRequirements(coroutine -> {
        coroutine.await(elevator.homingCommand());
        status = elevator.isPositionControlReady() ? "HOMING/COMPLETE" : "HOMING/FAILED";
        if (!elevator.isPositionControlReady()) stopRequested = true;
      }).named("Superstructure:Home");
      case READY_EMPTY, READY_HOLDING_CORAL -> holdDefault();
      case PREPARED_L2, PREPARED_L3, PREPARED_L4 -> holdPrepared(levelOf(state));
      case STOWING -> stowArm();
      case STORING_CORAL -> storeCoral();
      case SCANNING_CORAL -> scanSlots();
      case GETTING_BLACK_CORAL, GETTING_WHITE_CORAL -> getCoral(colorOf(state));
      case PREPARING_L2, PREPARING_L3, PREPARING_L4 -> prepareArm(levelOf(state));
      case PICKING_UP_L2, PICKING_UP_L3, PICKING_UP_L4 -> pickUpCoral(levelOf(state));
      case PLACING_L2, PLACING_L3, PLACING_L4 -> placeCoral(levelOf(state));
    };
  }

  private void stopMechanisms(Set<Mechanism> requirements) {
    if (requirements.contains(elevator)) elevator.stop();
    if (requirements.contains(indexer)) indexer.stop();
  }

  // These private flows only run under stateCommand, which reserves their complete requirements.
  // Successful finite motion retains its last position loop while the parent waits for the next step.

  private boolean awaitSafeMove(Coroutine coroutine, ArmPose pose) {
    ElevatorMoveSafelyCommand movement = new ElevatorMoveSafelyCommand(elevator, pose);
    coroutine.await(movement);
    if (movement.hasSucceeded()) return true;
    // await reports termination, so failed travel must explicitly abort the enclosing action.
    status = currentSuperState.name() + "/ELEVATOR_MOVE_FAILED";
    stopRequested = true;
    stopMechanisms(requirementsFor(currentSuperState));
    return false;
  }

  private Command holdDefault() {
    return Command.noRequirements(coroutine -> {
      requireAutomation(coroutine);
      if (!awaitSafeMove(coroutine, STOW)) return;
      coroutine.park();
    }).named("Superstructure:HoldStow");
  }

  private Command holdPrepared(Level level) {
    return Command.noRequirements(coroutine -> {
      requireAutomation(coroutine);
      if (selectedSide == null) block(coroutine, "NO_PREPARED_SIDE");
      ArmPose pose = preparation(level, endEffectorCoralState, selectedSide);
      if (!awaitSafeMove(coroutine, pose)) return;
      coroutine.park();
    }).named("Superstructure:HoldPrepared");
  }

  private Command stowArm() {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(coroutine);
      clearPreparation();
      if (!awaitSafeMove(coroutine, STOW)) return;
      completeAction();
    }).named("Superstructure:StowArm");
  }

  private Command prepareArm(Level level) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(coroutine);
      selectedLevel = null;
      selectedSide = null;
      if (endEffectorCoralState == CoralState.UNKNOWN) block(coroutine, "UNKNOWN_GRIPPER");
      var selection = ReefAlignCommand.selectReef(drive.getPose());
      if (selection.isEmpty()) block(coroutine, "NO_REEF_SIDE");
      RobotSide side = selection.orElseThrow().side();
      if (endEffectorCoralState == CoralState.EMPTY
          && !CoralHandlingConstants.positive(GRAB_ENTRY_HEIGHT_DROP.in(Meters))) {
        block(coroutine, "INVALID_GRAB_OFFSET");
      }
      ArmPose pose = preparation(level, endEffectorCoralState, side);
      if (!awaitSafeMove(coroutine, pose)) return;
      selectedLevel = level;
      selectedSide = side;
      completeAction();
    }).named("Superstructure:PrepareArm");
  }

  /** Raise clear, align one fixed slot, descend open, detect, clamp, and extract. */
  private Command getCoral(CoralState wanted) {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(coroutine);
      if (endEffectorCoralState != CoralState.EMPTY) block(coroutine, "GRIPPER_NOT_EMPTY");
      OptionalInt matchingSlot = findNearestSlot(wanted);
      if (matchingSlot.isEmpty()) block(coroutine, "NO_MATCHING_SLOT");
      int slot = matchingSlot.orElseThrow();
      clearPreparation();

      status = currentSuperState.name() + "/CLEARING_INDEXER";
      if (!awaitSafeMove(coroutine, STOW)) return;
      // Empty jaw opening and turntable alignment may overlap only after the arm is clear.
      var opening = coroutine.fork(jaw.openCommand());
      coroutine.await(indexer.rotateToSlotCommand(slot));
      opening.awaitCompletion();
      if (!awaitSafeMove(coroutine, INDEXER_PICKUP)) return;
      status = currentSuperState.name() + "/WAITING_FOR_CORAL";
      observeEndEffector(coroutine, Superstructure::isCoral);
      coroutine.waitUntil(() -> elevator.hasReached(INDEXER_PICKUP)
          && indexer.isSlotSettled(slot)
          && coralSensor.getEndEffectorObservation().filter(Superstructure::isCoral).isPresent());

      // From the actuation request until post-retreat confirmation, neither record is certain.
      setSlotCoralState(slot, CoralState.UNKNOWN);
      endEffectorCoralState = CoralState.UNKNOWN;
      coroutine.await(jaw.clampCommand());
      if (!awaitSafeMove(coroutine, STOW)) return;
      status = currentSuperState.name() + "/CONFIRMING_PICKUP";
      CoralState grabbed = observeEndEffector(coroutine, Superstructure::isCoral);
      endEffectorCoralState = grabbed;
      setSlotCoralState(slot, CoralState.EMPTY);
      completeAction();
    }).named("Superstructure:GetCoral");
  }

  private Command storeCoral() {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(coroutine);
      CoralState held = endEffectorCoralState;
      if (!isCoral(held)) block(coroutine, "NO_CONFIRMED_CORAL");
      OptionalInt emptySlot = findNearestSlot(CoralState.EMPTY);
      if (emptySlot.isEmpty()) block(coroutine, "NO_EMPTY_SLOT");
      int slot = emptySlot.orElseThrow();
      clearPreparation();

      status = currentSuperState.name() + "/CLEARING_INDEXER";
      if (!awaitSafeMove(coroutine, STOW)) return;
      coroutine.await(indexer.rotateToSlotCommand(slot));
      if (!awaitSafeMove(coroutine, INDEXER_STORE)) return;
      coroutine.waitUntil(() -> elevator.hasReached(INDEXER_STORE) && indexer.isSlotSettled(slot));
      setSlotCoralState(slot, CoralState.UNKNOWN);
      endEffectorCoralState = CoralState.UNKNOWN;
      status = currentSuperState.name() + "/RELEASING";
      coroutine.await(jaw.openCommand());
      if (!awaitSafeMove(coroutine, STOW)) return;
      status = currentSuperState.name() + "/CONFIRMING_RELEASE";
      observeEndEffector(coroutine, state -> state == CoralState.EMPTY);
      setSlotCoralState(slot, held);
      endEffectorCoralState = CoralState.EMPTY;
      completeAction();
    }).named("Superstructure:StoreCoral");
  }

  /** Scan progress belongs to this invocation; interrupted scans never commit partial inventory. */
  private Command scanSlots() {
    return Command.noRequirements(coroutine -> {
      indexer.stop();
      requireAutomation(coroutine);
      clearPreparation();
      if (!InventoryScanConstants.turntableConfigured()) block(coroutine, "SCAN_UNCONFIGURED");
      ArmPose clearance = new ArmPose(
          Meters.of(Math.max(elevator.getHeight().in(Meters),
              CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
                  + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters)
                  + ElevatorConstants.POSITION_TOLERANCE.in(Meters))),
          elevator.getPivotAngle());
      if (!awaitSafeMove(coroutine, clearance)) return;
      CoralState[] scanned = new CoralState[SpinIndexerConstants.SLOT_COUNT];
      for (int slot = 0; slot < scanned.length; slot++) {
        status = currentSuperState.name() + "/SLOT_" + (slot + 1);
        Angle target = InventoryScanConstants.slotAngle(slot);
        coroutine.await(indexer.rotateToAngleCommand(target,
            InventoryScanConstants.ALIGNMENT_TOLERANCE, InventoryScanConstants.SETTLE_SECONDS));
        waitForSensorFlush(coroutine);
        coralSensor.resetIndexerObservation();
        coroutine.waitUntil(() -> coralSensor.getIndexerObservation().isPresent());
        scanned[slot] = coralSensor.getIndexerObservation().orElseThrow();
      }
      for (int slot = 0; slot < scanned.length; slot++) setSlotCoralState(slot, scanned[slot]);
      completeAction();
    }).named("Superstructure:ScanSlots");
  }

  private Command pickUpCoral(Level level) {
    return Command.noRequirements(coroutine -> {
      drive.stop();
      requireAutomation(coroutine);
      if (endEffectorCoralState != CoralState.EMPTY) block(coroutine, "GRIPPER_NOT_EMPTY");
      AlignmentTarget target = alignment;
      if (target == null || selectedSide != target.side()) block(coroutine, "NO_MATCHING_ALIGNMENT");
      ArmPose entry = preparation(level, CoralState.EMPTY, target.side());
      ArmPose atBranch = grabPose(level, target.side());
      ArmPose away = new ArmPose(atBranch.height(), reefPreparationAngle(target.side()));
      if (!CoralHandlingConstants.positive(GRAB_ENTRY_HEIGHT_DROP.in(Meters))
          || !entry.configured() || !atBranch.configured() || !away.configured()) {
        block(coroutine, "INVALID_REEF_POSES");
      }

      if (!awaitSafeMove(coroutine, entry)) return;
      coroutine.await(jaw.openCommand());
      status = currentSuperState.name() + "/WAITING_FOR_ALIGNMENT";
      coroutine.waitUntil(() -> reefPoseReached(target));
      coroutine.await(elevator.moveToCommand(atBranch));
      status = currentSuperState.name() + "/WAITING_FOR_CORAL";
      observeEndEffector(coroutine, Superstructure::isCoral);
      coroutine.waitUntil(() -> reefPoseReached(target) && elevator.hasReached(atBranch)
          && coralSensor.getEndEffectorObservation().filter(Superstructure::isCoral).isPresent());
      endEffectorCoralState = CoralState.UNKNOWN;
      status = currentSuperState.name() + "/CLAMPING";
      coroutine.await(jaw.clampCommand());
      if (!awaitSafeMove(coroutine, away)) return;
      status = currentSuperState.name() + "/CONFIRMING_PICKUP";
      endEffectorCoralState = observeEndEffector(coroutine, Superstructure::isCoral);
      completeAction();
    }).named("Superstructure:PickUpCoral");
  }

  private Command placeCoral(Level level) {
    return Command.noRequirements(coroutine -> {
      drive.stop();
      requireAutomation(coroutine);
      if (!isCoral(endEffectorCoralState)) block(coroutine, "NO_CONFIRMED_CORAL");
      AlignmentTarget target = alignment;
      if (target == null || selectedSide != target.side()) block(coroutine, "NO_MATCHING_ALIGNMENT");
      ArmPose atBranch = new ArmPose(reefHeight(level), reefPlaceAngle(target.side()));
      Distance exitHeight = Meters.of(atBranch.height().in(Meters) - PLACE_EXIT_HEIGHT_DROP.in(Meters));
      ArmPose clear = new ArmPose(exitHeight, VERTICAL_UP_PIVOT_ANGLE);
      if (!CoralHandlingConstants.positive(PLACE_EXIT_HEIGHT_DROP.in(Meters))
          || !atBranch.configured() || !clear.configured()
          || clear.height().in(Meters) < CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
              + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters)
              + ElevatorConstants.POSITION_TOLERANCE.in(Meters)) {
        block(coroutine, "INVALID_REEF_POSES");
      }

      if (!awaitSafeMove(coroutine, atBranch)) return;
      status = currentSuperState.name() + "/WAITING_FOR_ALIGNMENT";
      coroutine.waitUntil(() -> reefPoseReached(target) && elevator.hasReached(atBranch));
      endEffectorCoralState = CoralState.UNKNOWN;
      status = currentSuperState.name() + "/RELEASING";
      coroutine.await(jaw.openCommand());
      // This calibrated Reef exit intentionally lowers and pivots concurrently.
      coroutine.await(elevator.moveToCommand(clear));
      status = currentSuperState.name() + "/CONFIRMING_RELEASE";
      observeEndEffector(coroutine, state -> state == CoralState.EMPTY);
      endEffectorCoralState = CoralState.EMPTY;
      completeAction();
    }).named("Superstructure:PlaceCoral");
  }

  private void requireAutomation(Coroutine coroutine) {
    if (!CoralHandlingConstants.AUTOMATION_CONFIGURED
        || !CoralHandlingConstants.clearanceConfigured()) block(coroutine, "AUTOMATION_UNCONFIGURED");
  }

  private void clearPreparation() {
    selectedLevel = null;
    selectedSide = null;
    alignment = null;
  }

  private void block(Coroutine coroutine, String reason) {
    elevator.holdStill();
    status = currentSuperState.name() + "/" + reason;
    // Keep the action and its requirements alive until an operator transition cancels it.
    coroutine.park();
  }

  private void completeAction() {
    status = currentSuperState.name() + "/COMPLETE";
  }

  private void waitForSensorFlush(Coroutine coroutine) {
    if (!CoralHandlingConstants.positive(InventoryScanConstants.SENSOR_FLUSH_SECONDS)) {
      block(coroutine, "SENSOR_FLUSH_UNCONFIGURED");
    }
    coroutine.wait(Seconds.of(InventoryScanConstants.SENSOR_FLUSH_SECONDS));
  }

  /** Discard in-motion/integration samples and require a new stable reading at this pose. */
  private CoralState observeEndEffector(Coroutine coroutine, Predicate<CoralState> accepted) {
    waitForSensorFlush(coroutine);
    coralSensor.resetEndEffectorObservation();
    coroutine.waitUntil(() -> coralSensor.getEndEffectorObservation().filter(accepted).isPresent());
    return coralSensor.getEndEffectorObservation().orElseThrow();
  }

  private static boolean isCoral(CoralState state) {
    return state == CoralState.BLACK || state == CoralState.WHITE;
  }

  // ========== Shared helpers ==========

  /**
   * True when the chassis sits on the last aligned reef pose within the configured tolerances —
   * 2910's isReadyToEject idea: the chassis must be on the branch before grabbing from it or
   * releasing onto it.
   */
  private boolean reefPoseReached(AlignmentTarget target) {
    return target != null && target.equals(alignment)
        && drive.isWithinPose(
            target.pose(), REEF_POSITION_TOLERANCE_METERS, REEF_HEADING_TOLERANCE_RADIANS);
  }

  /** Finds the shortest rotation to slots with matching contents; EMPTY can select storage. */
  private OptionalInt findNearestSlot(CoralState state) {
    Objects.requireNonNull(state, "state");
    if (!indexer.isPositionControlReady()) {
      return OptionalInt.empty();
    }
    int nearestSlot = -1;
    double shortestDistance = Double.POSITIVE_INFINITY;
    for (int slot = 0; slot < SpinIndexerConstants.SLOT_COUNT; slot++) {
      if (getSlotCoralState(slot) != state) {
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
      return new ArmPose(Meters.of(height.in(Meters)
          - GRAB_ENTRY_HEIGHT_DROP.in(Meters)), reefPlaceAngle(side));
    }
    return new ArmPose(height, reefPreparationAngle(side));
  }

  /** The true grab pose where LT clamps: the parked empty height risen by the grab entry drop. */
  private static ArmPose grabPose(Level level, RobotSide side) {
    return new ArmPose(reefHeight(level), reefPlaceAngle(side));
  }

  private static SuperState pickupStateForLevel(Level level) {
    if (level == null) {
      return null;
    }
    return switch (level) {
      case L2 -> SuperState.PICKING_UP_L2;
      case L3 -> SuperState.PICKING_UP_L3;
      case L4 -> SuperState.PICKING_UP_L4;
      default -> null;
    };
  }

  private static SuperState placeStateForLevel(Level level) {
    if (level == null) {
      return null;
    }
    return switch (level) {
      case L2 -> SuperState.PLACING_L2;
      case L3 -> SuperState.PLACING_L3;
      case L4 -> SuperState.PLACING_L4;
      default -> null;
    };
  }

  private static Level levelOf(SuperState state) {
    return switch (state) {
      case PREPARED_L2, PREPARING_L2, PICKING_UP_L2, PLACING_L2 -> Level.L2;
      case PREPARED_L3, PREPARING_L3, PICKING_UP_L3, PLACING_L3 -> Level.L3;
      case PREPARED_L4, PREPARING_L4, PICKING_UP_L4, PLACING_L4 -> Level.L4;
      default -> null;
    };
  }

  private static CoralState colorOf(SuperState state) {
    return state == SuperState.GETTING_BLACK_CORAL ? CoralState.BLACK : CoralState.WHITE;
  }

  private static boolean isActionState(SuperState state) {
    return state == SuperState.HOMING
        || state == SuperState.STOWING
        || state == SuperState.SCANNING_CORAL
        || isTransfer(state)
        || isPreparing(state)
        || isReefAction(state);
  }

  private static boolean isGetting(SuperState state) {
    return state == SuperState.GETTING_BLACK_CORAL || state == SuperState.GETTING_WHITE_CORAL;
  }

  private static boolean isStoring(SuperState state) {
    return state == SuperState.STORING_CORAL;
  }

  private static boolean isTransfer(SuperState state) {
    return isGetting(state) || isStoring(state);
  }

  private static boolean isPreparing(SuperState state) {
    return state == SuperState.PREPARING_L2
        || state == SuperState.PREPARING_L3
        || state == SuperState.PREPARING_L4;
  }

  private static boolean isPickingUp(SuperState state) {
    return state == SuperState.PICKING_UP_L2
        || state == SuperState.PICKING_UP_L3
        || state == SuperState.PICKING_UP_L4;
  }

  private static boolean isReefAction(SuperState state) {
    return isPickingUp(state)
        || state == SuperState.PLACING_L2
        || state == SuperState.PLACING_L3
        || state == SuperState.PLACING_L4;
  }
}
