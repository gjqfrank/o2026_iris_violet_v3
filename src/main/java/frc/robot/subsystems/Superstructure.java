// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems;

import java.util.Objects;
import java.util.EnumMap;
import java.util.OptionalInt;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Pose2d;
import static org.wpilib.units.Units.Meters;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.command3.Trigger;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.commands.ReefAlignCommand.AlignmentTarget;
import frc.robot.constants.CoralHandlingConstants;
import static frc.robot.constants.CoralHandlingConstants.CLAMP_WAIT_SECONDS;
import static frc.robot.constants.CoralHandlingConstants.INDEXER_PICKUP;
import static frc.robot.constants.CoralHandlingConstants.INDEXER_STORE;
import static frc.robot.constants.CoralHandlingConstants.OPEN_WAIT_SECONDS;
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
import frc.robot.subsystems.elevator.ElevatorConstants.ArmPose;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.spinindexer.CoralState;
import frc.robot.subsystems.spinindexer.SpinIndexer;
import frc.robot.subsystems.spinindexer.SpinIndexerConstants;
import frc.robot.util.ReefTargetColors.Level;

/**
 * Coral superstructure implemented as a Commands v3 {@link StateMachine}. Each {@link SuperState}
 * is registered as a machine state. The gamepad lives here as well: its button triggers define
 * direct {@code switchTo(...).when(...)} edges between machine states, with no parallel requested
 * state. Action states complete naturally and use {@code whenComplete()} to return to the
 * inventory-appropriate ready/held/prepared state. Phases are derived from mechanism feedback:
 * elevator arrivals, jaw valve readback, and turntable settling.
 *
 * <p>This class also owns the robot's global coral records — slot1CoralState, slot2CoralState,
 * slot3CoralState and endEffectorCoralState, plus the operator context selectedLevel,
 * selectedSide and alignment. Actions update only these records; no other copy of the coral
 * state exists anywhere in the code. Every record is logged every cycle, and an action that
 * finishes leaves every record null that no longer describes reality.
 *
 * <p>Gates hold an action at its current step until conditions are met, 2910-style: the jaw must
 * be free before taking a coral and holding one before storing or placing it, and the chassis
 * must sit on the aligned reef pose — with the arm at the branch — before grabbing from or
 * releasing onto it. Default states actively hold the arm at their canonical pose every cycle.
 */
public final class Superstructure {
  public enum SuperState {
    STOPPED,
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
  private final StateMachine stateMachine = new StateMachine("Superstructure");
  private final EnumMap<SuperState, State> states =
      new EnumMap<>(SuperState.class);

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

  // The only phase memory left: when the jaw valve was last commanded. The arm travel
  // (rise, swing, lower) runs inside the elevator; this class only names poses and goals.
  private double jawCommandedSince;

  // Scan progress: which turntable slot is being read, when its optical detection was last
  // restarted, and what each slot has confirmed so far.
  private int scanSlot = -1;
  private double scanResetSince = Double.NaN;
  private final CoralState[] scannedSlots = new CoralState[SpinIndexerConstants.SLOT_COUNT];

  public Superstructure(
      Drive drive, Elevator elevator, SpinIndexer indexer, EndEffector jaw, CoralSensor coralSensor) {
    this.drive = drive;
    this.elevator = elevator;
    this.indexer = indexer;
    this.jaw = jaw;
    this.coralSensor = coralSensor;

    for (SuperState state : SuperState.values()) {
      State machineState = stateMachine.addState(stateCommand(state));
      machineState.onEnter(() -> enterState(state));
      states.put(state, machineState);
    }
    stateMachine.setInitialState(states.get(defaultCurrentState()));
    for (SuperState from : SuperState.values()) {
      State fromState = states.get(from);
      fromState.switchTo(states.get(SuperState.STOPPED)).when(() -> stopRequested);
      addGamepadTransitions(fromState, from);
      if (isReadyState(from)) {
        fromState.switchTo(() -> states.get(defaultCurrentState()))
            .when(() -> defaultCurrentState() != currentSuperState);
      }
      if (from != SuperState.STOPPED) {
        fromState.switchTo(() -> states.get(defaultCurrentState())).whenComplete();
      }
    }
    gamepad.view().onTrue(
        Command.noRequirements(
                coroutine -> {
                  if (!isBusy()) {
                    coroutine.await(elevator.homingCommand());
                  }
                })
            .named("Elevator:HomingWhenIdle"));
    Scheduler.getDefault().schedule(stateMachine);
  }

  private void addGamepadTransitions(State from, SuperState sourceState) {
    transition(from, sourceState, SuperState.SCANNING_CORAL, scanButton);
    transition(from, sourceState, SuperState.GETTING_BLACK_CORAL, getBlackButton);
    transition(from, sourceState, SuperState.GETTING_WHITE_CORAL, getWhiteButton);
    from.switchTo(() -> states.get(stowOrStoreState()))
        .when(() -> stowOrStoreState() != sourceState && stowOrStoreButton.getAsBoolean());
    transition(from, sourceState, SuperState.PREPARING_L2, prepareL2Button);
    transition(from, sourceState, SuperState.PREPARING_L3, prepareL3Button);
    transition(from, sourceState, SuperState.PREPARING_L4, prepareL4Button);
    from.switchTo(() -> states.get(pickupStateForLevel(selectedLevel)))
      .when(() -> pickupStateForLevel(selectedLevel) != null
          && pickupStateForLevel(selectedLevel) != sourceState
          && pickupButton.getAsBoolean());
    from.switchTo(() -> states.get(placeStateForLevel(selectedLevel)))
      .when(() -> placeStateForLevel(selectedLevel) != null
          && placeStateForLevel(selectedLevel) != sourceState
          && placeButton.getAsBoolean());
  }

  private void transition(State fromState, SuperState from, SuperState to, Trigger button) {
    if (from != to) {
      fromState.switchTo(states.get(to))
          .when(() -> currentSuperState == from && button.getAsBoolean());
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
    drive.stop();
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
    if (currentSuperState == state) {
      return;
    }
    previousSuperState = currentSuperState;
    currentSuperState = state;
    if (state == SuperState.STOPPED) {
      stopRequested = false;
    }
    if (isActionState(state)) {
      status = state.name();
    }
  }

  private Command stateCommand(SuperState state) {
    Set<Mechanism> requirements = requirementsFor(state);
    return Command.requiring(requirements)
        .executing(
            coroutine -> {
              enterState(state);
              while (!applyState(state)) {
                coroutine.yield();
              }
            })
        .whenCanceled(this::stopMechanisms)
        .named("Superstructure:" + state.name());
  }

  private Set<Mechanism> requirementsFor(SuperState state) {
    return switch (state) {
      case STOPPED -> Set.of(drive, elevator, indexer);
      case READY_EMPTY, READY_HOLDING_CORAL, PREPARED_L2, PREPARED_L3, PREPARED_L4 ->
          Set.of(elevator);
      case STOWING, PREPARING_L2, PREPARING_L3, PREPARING_L4 -> Set.of(elevator, indexer);
      case SCANNING_CORAL -> Set.of(elevator, indexer, coralSensor);
      case STORING_CORAL, GETTING_BLACK_CORAL, GETTING_WHITE_CORAL ->
          Set.of(elevator, indexer, jaw, coralSensor);
      case PICKING_UP_L2, PICKING_UP_L3, PICKING_UP_L4,
          PLACING_L2, PLACING_L3, PLACING_L4 -> Set.of(drive, elevator, jaw, coralSensor);
    };
  }

  private boolean applyState(SuperState state) {
    return switch (state) {
      case STOPPED -> { // every mechanism halted, re-commanded each cycle until preempted
        drive.stop();
        elevator.stop();
        indexer.stop();
        yield false;
      }
      case READY_EMPTY, READY_HOLDING_CORAL -> { holdDefault(); yield false; }
      case PREPARED_L2, PREPARED_L3, PREPARED_L4 -> { holdPrepared(); yield false; }
      case STOWING -> stowArm();
      case STORING_CORAL -> storeCoral();
      case SCANNING_CORAL -> scanSlots();
      case GETTING_BLACK_CORAL, GETTING_WHITE_CORAL -> getCoral();
      case PREPARING_L2, PREPARING_L3, PREPARING_L4 -> prepareArm();
      case PICKING_UP_L2, PICKING_UP_L3, PICKING_UP_L4 -> pickUpCoral();
      case PLACING_L2, PLACING_L3, PLACING_L4 -> placeCoral();
    };
  }

  private void stopMechanisms() {
    drive.stop();
    elevator.stop();
    indexer.stop();
  }

  // ========== What each state does, every cycle ==========

  /** DEFAULT with no prepared level: hold the arm at the stow pose, 2910's noPiece/holdingCoral. */
  private void holdDefault() {
    elevator.moveSafelyTo(STOW);
  }

  /** DEFAULT while a level is prepared: hold the arm at that level's working pose. */
  private void holdPrepared() {
    elevator.moveSafelyTo(
        preparation(levelOf(currentSuperState), endEffectorCoralState, selectedSide));
  }

  /** STOWING: fold the arm back to the stow pose. */
  private boolean stowArm() {
    selectedLevel = null;
    selectedSide = null;
    alignment = null; // a non-reef action leaves no reef context behind
    indexer.stop();
    elevator.moveSafelyTo(STOW);
    if (elevator.hasReached(STOW)) {
      status = currentSuperState.name() + "/COMPLETE";
      return true;
    }
    return false;
  }

  /** PREPARING_Lx: travel safely to the level's working pose and remember the choice. */
  private boolean prepareArm() {
    selectedLevel = null; // re-earned when the travel completes
    selectedSide = null;
    indexer.stop();
    Level level = levelOf(currentSuperState);
    RobotSide side = ReefAlignCommand.selectReef(drive.getPose())
        .map(selection -> selection.side())
        .orElse(null);
    ArmPose prepPose = preparation(level, endEffectorCoralState, side);
    elevator.moveSafelyTo(prepPose);
    if (elevator.hasReached(prepPose)) {
      selectedLevel = level;
      selectedSide = side;
      status = currentSuperState.name() + "/COMPLETE";
      return true;
    }
    return false;
  }

  /** GETTING_BLACK/WHITE_CORAL: dip into the matching slot and pull its coral out into the jaw. */
  private boolean getCoral() {
    if (endEffectorCoralState != CoralState.EMPTY) { // holding one already; another would drop it
      elevator.holdStill();
      return false;
    }
    OptionalInt matchingSlot = findNearestSlot(colorOf(currentSuperState));
    if (matchingSlot.isEmpty()) {
      indexer.stop();
      elevator.holdStill();
      status = currentSuperState.name() + "/NO_MATCHING_SLOT";
      return false;
    }
    selectedLevel = null;
    selectedSide = null;
    alignment = null; // a non-reef action leaves no reef context behind
    int slot = matchingSlot.getAsInt();
    ArmPose contact = INDEXER_PICKUP;
    if (indexer.isSlotSettled(slot)) {
      indexer.holdCurrentPosition(); // hold the turntable on the slot
    } else if (indexer.isSafeToRotate()) {
      indexer.rotateToSlot(slot); // turn only once the arm is above the turntable
    }
    if (!indexer.isSlotSettled(slot)) {
      elevator.moveSafelyTo(STOW); // rise out of the turntable's way while it turns
    } else if (!jaw.isGripperClamped()) {
      elevator.moveSafelyTo(contact); // swing over the slot and dip in
      jaw.openGripper();
      coralSensor.resetEndEffectorObservation();
      if (elevator.hasReached(contact)) {
        jaw.clampGripper(); // down in the slot: grab the coral
        coralSensor.resetEndEffectorObservation();
        jawCommandedSince = Timer.getTimestamp();
      }
    } else if (Timer.getTimestamp() - jawCommandedSince < CLAMP_WAIT_SECONDS) {
      elevator.setTargets(contact); // hold still while the jaw closes
    } else {
      elevator.moveSafelyTo(STOW); // pull back out
      var color = coralSensor.getEndEffectorObservation()
          .filter(state -> state == CoralState.BLACK || state == CoralState.WHITE);
      if (elevator.hasReached(STOW) && color.isPresent()) {
        endEffectorCoralState = color.get(); // book the color actually grabbed
        setSlotCoralState(slot, CoralState.EMPTY);
        status = currentSuperState.name() + "/COMPLETE";
        return true;
      }
    }
    return false;
  }

  /** STORING_CORAL: dip into the nearest empty slot and drop the jaw's coral into it. */
  private boolean storeCoral() {
    if (endEffectorCoralState == CoralState.EMPTY) { // nothing in the jaw to store
      elevator.holdStill();
      return false;
    }
    OptionalInt emptySlot = findNearestSlot(CoralState.EMPTY);
    if (emptySlot.isEmpty()) {
      indexer.stop();
      elevator.holdStill();
      status = currentSuperState.name() + "/NO_EMPTY_SLOT";
      return false;
    }
    selectedLevel = null;
    selectedSide = null;
    alignment = null; // a non-reef action leaves no reef context behind
    int slot = emptySlot.getAsInt();
    ArmPose contact = INDEXER_STORE;
    if (indexer.isSlotSettled(slot)) {
      indexer.holdCurrentPosition(); // hold the turntable on the slot
    } else if (indexer.isSafeToRotate()) {
      indexer.rotateToSlot(slot); // turn only once the arm is above the turntable
    }
    if (!indexer.isSlotSettled(slot)) {
      elevator.moveSafelyTo(STOW); // rise out of the turntable's way while it turns
    } else if (!jaw.isGripperOpen()) {
      elevator.moveSafelyTo(contact); // carry the held coral over the slot and dip in
      if (elevator.hasReached(contact)) {
        jaw.openGripper(); // down in the slot: let go
        coralSensor.resetEndEffectorObservation();
        jawCommandedSince = Timer.getTimestamp();
      }
    } else if (Timer.getTimestamp() - jawCommandedSince < OPEN_WAIT_SECONDS) {
      elevator.setTargets(contact); // hold still while the jaw opens
    } else {
      elevator.moveSafelyTo(STOW); // pull back out
      if (elevator.hasReached(STOW)
          && coralSensor.getEndEffectorObservation().orElse(CoralState.UNKNOWN) == CoralState.EMPTY) {
        setSlotCoralState(slot, endEffectorCoralState); // book it into the slot first
        endEffectorCoralState = CoralState.EMPTY;
        status = currentSuperState.name() + "/COMPLETE";
        return true;
      }
    }
    return false;
  }

  /**
   * SCANNING_CORAL: hold the arm clear of the turntable at the current pivot, then read every
   * slot with the fixed turntable sensor; books what is confirmed instead of operator preloads.
   */
  private boolean scanSlots() {
    selectedLevel = null;
    selectedSide = null;
    alignment = null; // a non-reef action leaves no reef context behind
    // Clearance at the current pivot: never lower the arm, only lift it out of the turntable's way.
    ArmPose clearance = new ArmPose(
        Meters.of(Math.max(elevator.getHeight().in(Meters),
            CoralHandlingConstants.INDEXER_MIN_SAFE_HEIGHT.in(Meters)
                + CoralHandlingConstants.INDEXER_HEIGHT_MARGIN.in(Meters))),
        elevator.getPivotAngle());
    elevator.moveSafelyTo(clearance);
    if (!elevator.hasReached(clearance)) {
      scanResetSince = Double.NaN; // the reading restarts once the arm stops travelling
      return false;
    }
    if (!InventoryScanConstants.turntableConfigured()
        || !indexer.isPositionControlReady()) {
      return false; // hold: the slot readings cannot be trusted yet
    }
    if (scanSlot >= SpinIndexerConstants.SLOT_COUNT) { // every slot read: commit the scan
      slot1CoralState = scannedSlots[0];
      slot2CoralState = scannedSlots[1];
      slot3CoralState = scannedSlots[2];
      indexer.holdCurrentPosition();
      scanResetSince = Double.NaN;
      scanSlot = -1;
      status = currentSuperState.name() + "/COMPLETE";
      return true;
    }
    Angle target = InventoryScanConstants.slotAngle(scanSlot);
    if (!indexer.isAngleSettled(target, InventoryScanConstants.ALIGNMENT_TOLERANCE,
        InventoryScanConstants.SETTLE_SECONDS)) {
      scanResetSince = Double.NaN; // the reading restarts once the turntable settles
      if (indexer.isSafeToRotate()) {
        indexer.rotateToAngle(target);
      }
      return false;
    }
    if (!Double.isFinite(scanResetSince)) {
      coralSensor.resetIndexerObservation(); // discard anything seen during the rotation
      scanResetSince = Timer.getTimestamp();
      return false;
    }
    var observed = coralSensor.getIndexerObservation();
    if (observed.isEmpty()) {
      return false; // wait for the slot's stable fresh reading
    }
    scannedSlots[scanSlot] = observed.get();
    scanSlot++;
    scanResetSince = Double.NaN;
    return false;
  }

  /**
   * PICKING_UP_Lx: park the empty jaw a grab-entry drop below the branch, then rise into the
   * true grab height and clamp once the arm and the chassis are both in place — 2910's
   * arm-reached-setpoint plus drive-setpoint eject readiness.
   */
  private boolean pickUpCoral() {
    drive.stop();
    RobotSide side = alignment == null ? null : alignment.side();
    Level level = levelOf(currentSuperState);
    ArmPose entry = preparation(level, endEffectorCoralState, side); // parks below the grab height
    ArmPose atBranch = grabPose(level, side); // rises through the entry drop to clamp
    ArmPose away = new ArmPose(atBranch.height(), reefPreparationAngle(side));
    if (!jaw.isGripperClamped() && endEffectorCoralState != CoralState.EMPTY) {
      elevator.holdStill(); // already holding a coral: taking another would drop this one
    } else if (!jaw.isGripperClamped()) {
      // Once the final rise has started, keep targeting the branch instead of returning to entry.
      if (!elevator.isCommandedTo(atBranch) && !elevator.hasReached(entry)) {
        elevator.moveSafelyTo(entry); // carry the empty jaw toward the branch, below the grab height
        return false;
      }
      if (!reefPoseReached()) {
        elevator.setTargets(entry); // hold at the entry height until the chassis is on the pose
        return false;
      }
      if (!elevator.hasReached(atBranch)) {
        elevator.setTargets(atBranch); // chassis aligned: rise the last bit into the grab height
        return false;
      }
      jaw.clampGripper(); // arm at the grab height and chassis on the aligned pose: grab the coral
      coralSensor.resetEndEffectorObservation();
      jawCommandedSince = Timer.getTimestamp();
    } else if (Timer.getTimestamp() - jawCommandedSince < CLAMP_WAIT_SECONDS) {
      elevator.setTargets(atBranch); // hold still while the jaw closes
    } else {
      elevator.moveSafelyTo(away); // back the arm away from the branch
      var color = coralSensor.getEndEffectorObservation()
          .filter(state -> state == CoralState.BLACK || state == CoralState.WHITE);
      if (elevator.hasReached(away) && color.isPresent()) {
        endEffectorCoralState = color.get();
        status = currentSuperState.name() + "/COMPLETE";
        return true;
      }
    }
    return false;
  }

  /** PLACING_Lx: set the held coral onto the prepared reef branch, then descend-and-rotate clear. */
  private boolean placeCoral() {
    drive.stop();
    if (endEffectorCoralState == CoralState.EMPTY) { // nothing in the jaw to place
      elevator.holdStill();
      return false;
    }
    RobotSide side = alignment == null ? null : alignment.side();
    ArmPose atBranch =
        new ArmPose(
            preparation(levelOf(currentSuperState), endEffectorCoralState, side).height(),
            reefPlaceAngle(side));
    Distance exitHeight = Meters.of(atBranch.height().in(Meters) - PLACE_EXIT_HEIGHT_DROP.in(Meters));
    ArmPose clear = new ArmPose(exitHeight, VERTICAL_UP_PIVOT_ANGLE);
    if (!jaw.isGripperOpen()) {
      elevator.moveSafelyTo(atBranch); // carry the held coral to the branch
      if (elevator.hasReached(atBranch) && reefPoseReached()) {
        jaw.openGripper(); // arm at the branch and chassis on the aligned pose: let go
        coralSensor.resetEndEffectorObservation();
        jawCommandedSince = Timer.getTimestamp();
      }
    } else if (Timer.getTimestamp() - jawCommandedSince < OPEN_WAIT_SECONDS) {
      elevator.setTargets(atBranch); // hold still while the jaw opens
    } else if (elevator.isCommandedTo(clear)) {
      if (elevator.atTargets()
          && coralSensor.getEndEffectorObservation().orElse(CoralState.UNKNOWN) == CoralState.EMPTY) {
        endEffectorCoralState = CoralState.EMPTY; // the coral stayed on the branch
        status = currentSuperState.name() + "/COMPLETE";
        return true;
      }
    } else {
      elevator.setTargets(clear); // one motion: descend away from the branch while rotating up
    }
    return false;
  }

  // ========== Shared helpers ==========

  /**
   * True when the chassis sits on the last aligned reef pose within the configured tolerances —
   * 2910's isReadyToEject idea: the chassis must be on the branch before grabbing from it or
   * releasing onto it.
   */
  private boolean reefPoseReached() {
    return alignment != null
        && drive.isWithinPose(
            alignment.pose(), REEF_POSITION_TOLERANCE_METERS, REEF_HEADING_TOLERANCE_RADIANS);
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
    return state == SuperState.STOWING
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
