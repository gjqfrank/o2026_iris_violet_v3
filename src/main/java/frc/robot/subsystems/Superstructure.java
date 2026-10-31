// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.command3.Trigger;
import frc.robot.commands.ReefAlignCommand.AlignmentTarget;
import frc.robot.commands.SuperstructureCommandFactory;
import frc.robot.constants.CoralHandlingConstants;
import frc.robot.constants.ReefAlignConstants.RobotSide;
import frc.robot.subsystems.coralsensor.CoralSensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.spinindexer.CoralState;
import frc.robot.subsystems.spinindexer.SpinIndexer;
import frc.robot.util.ReefTargetColors.Level;

/**
 * Coral superstructure whose {@link #controlCommand()} builds a Commands v3 {@link StateMachine}.
 * The v3 machine is the only state representation: {@link SuperstructureCommandFactory}'s static
 * builders construct every state's action command from the mechanisms they command, and the state
 * table in {@code controlCommand()} pairs each state name with its owned mechanisms and action in
 * one place. Every state keeps its name — the name of the command bound to the node — which
 * telemetry reports as the current state. The gamepad
 * lives here as well: its button triggers define direct {@code switchTo(...).when(...)} edges
 * between machine states, with no parallel requested state. Action states complete naturally and
 * use {@code whenComplete()} to return to the inventory-appropriate ready/held/prepared state.
 *
 * <p>This class also owns the robot's global coral records — slot1CoralState, slot2CoralState,
 * slot3CoralState and endEffectorCoralState, plus the operator context selectedLevel,
 * selectedSide and alignment. The factory's actions update these records through the accessors
 * below; no other copy of the coral state exists anywhere in the code. Every record is logged
 * every cycle, and an action that finishes updates the records to reflect the confirmed
 * observations.
 *
 * <p>Gates hold an action at its current step until conditions are met, 2910-style: the jaw must
 * be free before taking a coral and holding one before storing or placing it, and the chassis
 * must sit on the aligned reef pose — with the arm at the branch — before grabbing from or
 * releasing onto it. State commands reserve all mechanisms needed by the complete action,
 * including sensor waits. Normal motion completion retains closed-loop holding; cancellation
 * stops owned motors and leaves the jaw valve unchanged.
 */
public final class Superstructure {
  /** State names that move mechanisms toward a goal; telemetry reports them as busy. */
  private static final Set<String> ACTION_STATES = Set.of(
      "HOMING", "STOWING", "SCANNING_CORAL",
      "STORING_CORAL", "GETTING_BLACK_CORAL", "GETTING_WHITE_CORAL",
      "PREPARING_L2", "PREPARING_L3", "PREPARING_L4",
      "PICKING_UP_L2", "PICKING_UP_L3", "PICKING_UP_L4",
      "PLACING_L2", "PLACING_L3", "PLACING_L4");

  /** Reef action state names that must lock out chassis driving. */
  private static final Set<String> REEF_ACTION_STATES = Set.of(
      "PICKING_UP_L2", "PICKING_UP_L3", "PICKING_UP_L4",
      "PLACING_L2", "PLACING_L3", "PLACING_L4");

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

  // Name of the state machine state entered most recently; the v3 machine itself owns the true
  // notion of "current state", these records only mirror it for telemetry and queries.
  private String currentState = "STOPPED";
  private String previousState = "STOPPED";
  private Set<Mechanism> currentRequirements = Set.of();
  private boolean stopRequested;
  private String status = "PRELOAD_REQUIRED";

  // ========== The robot's global records ==========
  // The only copies of the coral and operator state anywhere in the code. The factory's actions
  // write them through the accessors below; everything else reads them. All are logged every cycle.

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

  /**
   * Builds a fresh teleop state graph; the caller owns scheduling and cancellation. The state
   * table below is the single place that pairs every state name with its owned mechanisms and
   * action command from {@link SuperstructureCommandFactory}.
   */
  public Command controlCommand() {
    StateMachine machine = new StateMachine("Superstructure");

    // ===== The state table: every state's name -> owned mechanisms + action command =====
    Map<String, StateAction> table = Map.ofEntries(
        Map.entry("STOPPED", new StateAction(Set.of(elevator, indexer),
            SuperstructureCommandFactory.stoppedAction(elevator, indexer))),
        Map.entry("HOMING", new StateAction(Set.of(elevator),
            SuperstructureCommandFactory.homingAction(this, elevator))),
        Map.entry("READY_EMPTY", new StateAction(Set.of(elevator),
            SuperstructureCommandFactory.holdDefault(this, elevator))),
        Map.entry("READY_HOLDING_CORAL", new StateAction(Set.of(elevator),
            SuperstructureCommandFactory.holdDefault(this, elevator))),
        Map.entry("PREPARED_L2", new StateAction(Set.of(elevator),
            SuperstructureCommandFactory.holdPrepared(this, elevator, Level.L2))),
        Map.entry("PREPARED_L3", new StateAction(Set.of(elevator),
            SuperstructureCommandFactory.holdPrepared(this, elevator, Level.L3))),
        Map.entry("PREPARED_L4", new StateAction(Set.of(elevator),
            SuperstructureCommandFactory.holdPrepared(this, elevator, Level.L4))),
        Map.entry("STOWING", new StateAction(Set.of(elevator, indexer),
            SuperstructureCommandFactory.stowArm(this, elevator, indexer))),
        Map.entry("STORING_CORAL", new StateAction(Set.of(elevator, indexer, jaw, coralSensor),
            SuperstructureCommandFactory.storeCoral(this, elevator, indexer, jaw, coralSensor))),
        Map.entry("SCANNING_CORAL", new StateAction(Set.of(elevator, indexer, coralSensor),
            SuperstructureCommandFactory.scanSlots(this, elevator, indexer, coralSensor))),
        Map.entry("GETTING_BLACK_CORAL", new StateAction(
            Set.of(elevator, indexer, jaw, coralSensor),
            SuperstructureCommandFactory.getCoral(
                this, elevator, indexer, jaw, coralSensor, CoralState.BLACK))),
        Map.entry("GETTING_WHITE_CORAL", new StateAction(
            Set.of(elevator, indexer, jaw, coralSensor),
            SuperstructureCommandFactory.getCoral(
                this, elevator, indexer, jaw, coralSensor, CoralState.WHITE))),
        Map.entry("PREPARING_L2", new StateAction(Set.of(elevator, indexer),
            SuperstructureCommandFactory.prepareArm(this, drive, elevator, indexer, Level.L2))),
        Map.entry("PREPARING_L3", new StateAction(Set.of(elevator, indexer),
            SuperstructureCommandFactory.prepareArm(this, drive, elevator, indexer, Level.L3))),
        Map.entry("PREPARING_L4", new StateAction(Set.of(elevator, indexer),
            SuperstructureCommandFactory.prepareArm(this, drive, elevator, indexer, Level.L4))),
        Map.entry("PICKING_UP_L2", new StateAction(
            Set.of(drive, elevator, jaw, coralSensor),
            SuperstructureCommandFactory.pickUpCoral(
                this, drive, elevator, jaw, coralSensor, Level.L2))),
        Map.entry("PICKING_UP_L3", new StateAction(
            Set.of(drive, elevator, jaw, coralSensor),
            SuperstructureCommandFactory.pickUpCoral(
                this, drive, elevator, jaw, coralSensor, Level.L3))),
        Map.entry("PICKING_UP_L4", new StateAction(
            Set.of(drive, elevator, jaw, coralSensor),
            SuperstructureCommandFactory.pickUpCoral(
                this, drive, elevator, jaw, coralSensor, Level.L4))),
        Map.entry("PLACING_L2", new StateAction(
            Set.of(drive, elevator, jaw, coralSensor),
            SuperstructureCommandFactory.placeCoral(
                this, drive, elevator, jaw, coralSensor, Level.L2))),
        Map.entry("PLACING_L3", new StateAction(
            Set.of(drive, elevator, jaw, coralSensor),
            SuperstructureCommandFactory.placeCoral(
                this, drive, elevator, jaw, coralSensor, Level.L3))),
        Map.entry("PLACING_L4", new StateAction(
            Set.of(drive, elevator, jaw, coralSensor),
            SuperstructureCommandFactory.placeCoral(
                this, drive, elevator, jaw, coralSensor, Level.L4))));

    Map<String, State> states = new HashMap<>();
    table.forEach(
        (name, action) -> states.put(name, addState(machine, name, action.requirements(), action.action())));
    machine.setInitialState(states.get(defaultCurrentState()));

    // ===== Transitions, in evaluation order =====
    // 1. A requested stop preempts every state.
    machine.switchFromAny().to(states.get("STOPPED")).when(() -> stopRequested);

    // 2. Gamepad edges are available from every state except HOMING; a state never restarts itself.
    State[] gamepadStates = statesNamed(states,
        "STOPPED", "READY_EMPTY", "READY_HOLDING_CORAL", "PREPARED_L2", "PREPARED_L3", "PREPARED_L4",
        "STOWING", "STORING_CORAL", "SCANNING_CORAL", "GETTING_BLACK_CORAL", "GETTING_WHITE_CORAL",
        "PREPARING_L2", "PREPARING_L3", "PREPARING_L4",
        "PICKING_UP_L2", "PICKING_UP_L3", "PICKING_UP_L4",
        "PLACING_L2", "PLACING_L3", "PLACING_L4");
    machine.switchFromAny(gamepadStates).to(states.get("SCANNING_CORAL"))
        .when(() -> !inState("SCANNING_CORAL") && scanButton.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(states.get("GETTING_BLACK_CORAL"))
        .when(() -> !inState("GETTING_BLACK_CORAL") && getBlackButton.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(states.get("GETTING_WHITE_CORAL"))
        .when(() -> !inState("GETTING_WHITE_CORAL") && getWhiteButton.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(() -> states.get(stowOrStoreState()))
        .when(() -> !stowOrStoreState().equals(currentState) && stowOrStoreButton.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(states.get("PREPARING_L2"))
        .when(() -> !inState("PREPARING_L2") && prepareL2Button.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(states.get("PREPARING_L3"))
        .when(() -> !inState("PREPARING_L3") && prepareL3Button.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(states.get("PREPARING_L4"))
        .when(() -> !inState("PREPARING_L4") && prepareL4Button.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(() -> states.get(pickupStateForLevel(selectedLevel)))
        .when(() -> pickupStateForLevel(selectedLevel) != null
            && !pickupStateForLevel(selectedLevel).equals(currentState)
            && pickupButton.getAsBoolean());
    machine.switchFromAny(gamepadStates).to(() -> states.get(placeStateForLevel(selectedLevel)))
        .when(() -> placeStateForLevel(selectedLevel) != null
            && !placeStateForLevel(selectedLevel).equals(currentState)
            && placeButton.getAsBoolean());

    // 3. Homing is reachable from the stopped and ready states.
    machine.switchFromAny(statesNamed(states, "STOPPED", "READY_EMPTY", "READY_HOLDING_CORAL",
            "PREPARED_L2", "PREPARED_L3", "PREPARED_L4"))
        .to(states.get("HOMING")).when(homingButton);

    // 4. Ready states follow the inventory-appropriate default whenever it changes.
    machine.switchFromAny(statesNamed(states, "READY_EMPTY", "READY_HOLDING_CORAL",
            "PREPARED_L2", "PREPARED_L3", "PREPARED_L4"))
        .to(() -> states.get(defaultCurrentState()))
        .when(() -> !defaultCurrentState().equals(currentState));

    // 5. Completed actions return to the stopped or default ready state.
    machine.switchFromAny(statesNamed(states, "HOMING", "READY_EMPTY", "READY_HOLDING_CORAL",
            "PREPARED_L2", "PREPARED_L3", "PREPARED_L4", "STOWING", "STORING_CORAL", "SCANNING_CORAL",
            "GETTING_BLACK_CORAL", "GETTING_WHITE_CORAL", "PREPARING_L2", "PREPARING_L3",
            "PREPARING_L4", "PICKING_UP_L2", "PICKING_UP_L3", "PICKING_UP_L4",
            "PLACING_L2", "PLACING_L3", "PLACING_L4"))
        .to(() -> states.get(stopRequested ? "STOPPED" : defaultCurrentState()))
        .whenComplete();

    return Command.noRequirements(coroutine -> coroutine.await(machine))
        .whenExited(this::clearRuntimeState)
        .named("Superstructure:Teleop");
  }

  /** A state's owned mechanisms and action command, paired in the state table. */
  private record StateAction(Set<Mechanism> requirements, Command action) {}

  /** Adds a named state whose command reserves its mechanisms and records the entry. */
  private State addState(
      StateMachine machine, String name, Set<Mechanism> requirements, Command action) {
    Command command = Command.requiring(requirements)
        .executing(coroutine -> {
          // State.onEnter runs after the command's first execution, so initialize here.
          enterState(name, requirements);
          coroutine.await(action);
        })
        .whenCanceled(() -> stopMechanisms(requirements))
        .named("Superstructure:" + name);
    return machine.addState(command);
  }

  /** Resolves state names to their machine nodes for the transition groups above. */
  private static State[] statesNamed(Map<String, State> states, String... names) {
    State[] selected = new State[names.length];
    for (int i = 0; i < names.length; i++) {
      selected[i] = states.get(names[i]);
    }
    return selected;
  }

  private boolean inState(String name) {
    return name.equals(currentState);
  }

  public CommandXboxController gamepad() {
    return gamepad;
  }

  // ========== Record access ==========
  // The factory's action commands write the records through the mutators in this section; other
  // classes only read through the getters.

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

  /** The reef side of the last completed preparation, or null when none is active. */
  public RobotSide getSelectedSide() {
    return selectedSide;
  }

  /** The reef face targeted by the bumpers, or null when no alignment is active. */
  public AlignmentTarget getAlignment() {
    return alignment;
  }

  /** Slot indices are zero-based: 0, 1, 2 correspond to dashboard Slot1, Slot2, Slot3. */
  public void setSlotCoralState(int slot, CoralState state) {
    Objects.requireNonNull(state, "state");
    switch (slot) {
      case 0 -> slot1CoralState = state;
      case 1 -> slot2CoralState = state;
      case 2 -> slot3CoralState = state;
      default -> throw new IndexOutOfBoundsException("Coral slot must be 0, 1, or 2: " + slot);
    }
  }

  public void setEndEffectorCoralState(CoralState state) {
    endEffectorCoralState = Objects.requireNonNull(state, "state");
  }

  /** Records a completed preparation; pass nulls to invalidate a stale selection. */
  public void setPreparation(Level level, RobotSide side) {
    selectedLevel = level;
    selectedSide = side;
  }

  /** Forgets the preparation level, side, and bumper alignment. */
  public void clearPreparation() {
    selectedLevel = null;
    selectedSide = null;
    alignment = null;
  }

  /** Asks the machine to fall into STOPPED at the next transition check. */
  public void requestStop() {
    stopRequested = true;
  }

  /** Records a step within the current action, e.g. "PLACING_L2/RELEASING". */
  public void setStatusStep(String step) {
    status = currentState + "/" + step;
  }

  /** Halts the mechanisms owned by the currently entered state. */
  public void stopOwnedMechanisms() {
    stopMechanisms(currentRequirements);
  }

  /** Applies the four preload choosers and forgets any stale operator choices. */
  public void applyPreload(
      CoralState slot1, CoralState slot2, CoralState slot3, CoralState gripperCoral) {
    slot1CoralState = Objects.requireNonNull(slot1, "slot1");
    slot2CoralState = Objects.requireNonNull(slot2, "slot2");
    slot3CoralState = Objects.requireNonNull(slot3, "slot3");
    endEffectorCoralState = Objects.requireNonNull(gripperCoral, "gripperCoral");
    clearPreparation();
    // On the next state completion, the dynamic completion edge selects from the updated inventory.
    status = "PRELOAD_APPLIED";
  }

  public boolean isBusy() {
    return ACTION_STATES.contains(currentState);
  }

  public boolean isDriveLocked() {
    return REEF_ACTION_STATES.contains(currentState);
  }

  public void observeAlignment(AlignmentTarget target) {
    alignment = target;
  }

  /** True when the elevator is homed and calibrated, so position commands are meaningful. */
  public boolean isElevatorReady() {
    return elevator.isPositionControlReady();
  }

  /**
   * Stops every mechanism immediately and holds the STOPPED state — 2910's stopped() — until
   * another wanted state preempts it.
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
    Logger.recordOutput("Superstructure/CurrentSuperState", currentState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousState);
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

  /** The state to hold still in, ready to work with whatever the jaw currently has. */
  private String defaultCurrentState() {
    if (endEffectorCoralState == CoralState.UNKNOWN) return "STOPPED";
    if (selectedLevel != null && selectedSide != null) {
      return switch (selectedLevel) {
        case L2 -> "PREPARED_L2";
        case L3 -> "PREPARED_L3";
        case L4 -> "PREPARED_L4";
      };
    }
    return readyState();
  }

  private String readyState() {
    return endEffectorCoralState == CoralState.EMPTY
        ? "READY_EMPTY" : "READY_HOLDING_CORAL";
  }

  private String stowOrStoreState() {
    return endEffectorCoralState == CoralState.EMPTY ? "STOWING" : "STORING_CORAL";
  }

  private static String pickupStateForLevel(Level level) {
    if (level == null) return null;
    return switch (level) {
      case L2 -> "PICKING_UP_L2";
      case L3 -> "PICKING_UP_L3";
      case L4 -> "PICKING_UP_L4";
    };
  }

  private static String placeStateForLevel(Level level) {
    if (level == null) return null;
    return switch (level) {
      case L2 -> "PLACING_L2";
      case L3 -> "PLACING_L3";
      case L4 -> "PLACING_L4";
    };
  }

  private void enterState(String state, Set<Mechanism> requirements) {
    if (!state.equals(currentState)) {
      previousState = currentState;
      currentState = state;
    }
    currentRequirements = requirements;
    if (state.equals("STOPPED")) {
      stopRequested = false;
    }
    if (state.equals("HOMING")) {
      // Homing must not finish by returning to a previously prepared reef pose.
      clearPreparation();
    }
    if (ACTION_STATES.contains(state)) {
      status = state;
    }
  }

  private void clearRuntimeState() {
    previousState = currentState;
    currentState = "STOPPED";
    currentRequirements = Set.of();
    stopRequested = false;
    clearPreparation();
    status = "INACTIVE";
  }

  private void stopMechanisms(Set<Mechanism> requirements) {
    if (requirements.contains(elevator)) elevator.stop();
    if (requirements.contains(indexer)) indexer.stop();
  }
}
