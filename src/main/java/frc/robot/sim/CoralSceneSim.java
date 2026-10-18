// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.sim;

import static org.wpilib.units.Units.*;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;
import frc.robot.constants.CoralHandlingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.InventoryScanConstants;
import frc.robot.constants.ReefAlignConstants;
import frc.robot.constants.ReefAlignConstants.ReefBranch;
import frc.robot.constants.ReefAlignConstants.RobotSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ArmPose;
import frc.robot.subsystems.coralsensor.CANrangeIOSim;
import frc.robot.subsystems.coralsensor.ColorSensorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOSim.GripperRequest;
import frc.robot.subsystems.spinindexer.CoralState;
import frc.robot.subsystems.spinindexer.SpinIndexer;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.ReefTargetColors;
import frc.robot.util.ReefTargetColors.Level;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Discrete contact/retention model. Physical inventory never follows Command inventory commits. */
public final class CoralSceneSim {
  private record ReefSlot(int tag, Level level, ReefBranch branch) {}
  private record Contact(int indexerSlot, ReefSlot reefSlot, ArmPose arm, Pose2d robot) {}

  private final Drive drive;
  private final Elevator elevator;
  private final SpinIndexer indexer;
  private final EndEffectorIOSim jaw;
  private final CANrangeIOSim gripperRangeSensor;
  private final ColorSensorIOSim gripperColorSensor;
  private final CANrangeIOSim indexerRangeSensor;
  private final ColorSensorIOSim indexerColorSensor;
  private final ReefTargetColors targetColors;
  private final CoralState[] slots = {CoralState.EMPTY, CoralState.EMPTY, CoralState.EMPTY};
  private final Map<ReefSlot, CoralState> reef = new LinkedHashMap<>();
  private CoralState held = CoralState.EMPTY;
  private GripperRequest pendingRequest;
  private Contact pendingContact;
  private long lastRequestSequence;
  private int droppedCorals;
  private String lastTransfer = "NONE";
  private boolean reefInitialized;
  private final BooleanEntry coralSensorConnected = booleanSetting("Sim/CoralSensor/Connected", true);
  private final BooleanEntry spinIndexerSensorConnected = booleanSetting("Sim/SpinIndexerCoralSensor/Connected", true);
  private final BooleanEntry gripperRangeConnected = booleanSetting("Sim/GripperCANrange/Connected", true);
  private final BooleanEntry indexerRangeConnected = booleanSetting("Sim/IndexerCANrange/Connected", true);
  private final BooleanEntry coralTransfersEnabled = booleanSetting("Sim/CoralTransfersEnabled", true);

  public CoralSceneSim(Drive drive, Elevator elevator, SpinIndexer indexer,
      EndEffectorIOSim jaw, CANrangeIOSim gripperRangeSensor, ColorSensorIOSim gripperColorSensor,
      CANrangeIOSim indexerRangeSensor, ColorSensorIOSim indexerColorSensor,
      ReefTargetColors targetColors) {
    this.drive = drive;
    this.elevator = elevator;
    this.indexer = indexer;
    this.jaw = jaw;
    this.gripperRangeSensor = gripperRangeSensor;
    this.gripperColorSensor = gripperColorSensor;
    this.indexerRangeSensor = indexerRangeSensor;
    this.indexerColorSensor = indexerColorSensor;
    this.targetColors = targetColors;
    resetReef();
  }

  /** Explicit preload confirmation changes robot contents, without refilling the Reef. */
  public void loadRobot(CoralState slot1, CoralState slot2, CoralState slot3, CoralState gripper) {
    // The operator may confirm preloads before the first Teleop entry.
    if (!reefInitialized) resetReef();
    slots[0] = slot1;
    slots[1] = slot2;
    slots[2] = slot3;
    held = gripper;
    pendingRequest = null;
    pendingContact = null;
    GripperRequest request = jaw.getGripperRequest();
    lastRequestSequence = request == null ? 0 : request.sequence();
    updateSensor();
  }

  /** Both BACK SIDEs start with reversed L4 colors; all other rods are empty. */
  public void resetReef() {
    pendingRequest = null;
    pendingContact = null;
    reef.clear();
    for (int tag : FieldConstants.ALL_REEF_TAG_IDS.stream().sorted().toList()) {
      for (Level level : Level.values()) {
        for (ReefBranch branch : ReefBranch.values()) {
          CoralState color = CoralState.EMPTY;
            if ((tag == FieldConstants.BLUE_BACK_SIDE_TAG_ID
              || tag == FieldConstants.RED_BACK_SIDE_TAG_ID)
              && level == Level.L4) {
            color = targetColors.getTargetColor(level, branch)
                .map(value -> value == CoralState.BLACK ? CoralState.WHITE : CoralState.BLACK)
                .orElse(CoralState.EMPTY);
          }
          reef.put(new ReefSlot(tag, level, branch), color);
        }
      }
    }
    droppedCorals = 0;
    lastTransfer = "SCENE_RESET";
    reefInitialized = targetColors.getTargetColor(Level.L4, ReefBranch.LEFT).isPresent();
    Logger.recordOutput("Sim/Coral/ReefInitialized", reefInitialized);
  }

  public void update() {
    GripperRequest request = jaw.getGripperRequest();
    if (request != null && request.sequence() != lastRequestSequence) {
      lastRequestSequence = request.sequence();
      pendingRequest = request;
      pendingContact = findContact(request.open());
    }
    // Require contact throughout the cylinder stroke, rather than granting a transfer on request.
    if (pendingContact != null && !atContact(pendingContact)) pendingContact = null;
    if (pendingRequest != null
        && Timer.getTimestamp() - pendingRequest.timestamp() >= SimConstants.JAW_ACTUATION_SECONDS) {
      completeJawMotion();
      pendingRequest = null;
      pendingContact = null;
    }
    updateSensor();
    Logger.recordOutput("Sim/Coral/Gripper", held.name());
    Logger.recordOutput("Sim/Coral/Indexer", Arrays.stream(slots).map(Enum::name).toArray(String[]::new));
    Logger.recordOutput("Sim/Coral/LastTransfer", lastTransfer);
    Logger.recordOutput("Sim/Coral/DroppedCount", droppedCorals);
    reef.forEach((slot, color) -> Logger.recordOutput(
        "Sim/Coral/Reef/" + slot.tag() + "/" + slot.level() + "/" + slot.branch(), color.name()));
  }

  private void completeJawMotion() {
    if (!coralTransfersEnabled.get()) {
      lastTransfer = "TRANSFER_DISABLED";
      return;
    }
    if (pendingRequest.open()) {
      if (held == CoralState.EMPTY) return;
      if (pendingContact == null) {
        // An unsupported release drops the Coral; EMPTY alone cannot prove a successful placement.
        droppedCorals++;
        held = CoralState.EMPTY;
        lastTransfer = "DROPPED_NO_CONTACT";
      } else if (contents(pendingContact) == CoralState.EMPTY) {
        setContents(pendingContact, held);
        held = CoralState.EMPTY;
        lastTransfer = "PLACED";
      } else {
        // Ideal obstruction model: an occupied receiver prevents successful release.
        lastTransfer = "RECEIVER_OCCUPIED";
      }
    } else if (held == CoralState.EMPTY) {
      if (pendingContact != null && contents(pendingContact) != CoralState.EMPTY) {
        held = contents(pendingContact);
        setContents(pendingContact, CoralState.EMPTY);
        lastTransfer = "PICKED_UP";
      } else {
        lastTransfer = "PICKUP_MISSED";
      }
    }
  }

  private Contact findContact(boolean opening) {
    ArmPose indexerPose = opening ? CoralHandlingConstants.INDEXER_STORE
        : CoralHandlingConstants.INDEXER_PICKUP;
    var aligned = indexer.getAlignedSlot();
    if (aligned.isPresent()) {
      Contact contact = new Contact(aligned.getAsInt(), null, indexerPose, null);
      if (atContact(contact)) return contact;
    }
    for (ReefSlot slot : reef.keySet()) {
      for (RobotSide side : RobotSide.values()) {
        var offset = ReefAlignConstants.targetOffset(side, slot.branch());
        if (offset == null) continue;
        Pose2d robot = VisionConstants.aprilTagLayout.getTagPose(slot.tag()).orElseThrow()
            .toPose2d().transformBy(new Transform2d(offset,
                Rotation2d.fromRadians(side == RobotSide.LEFT ? Math.PI / 2 : -Math.PI / 2)));
        // Physical grab/place contact stays at the base height; the offsets only affect approach/retreat.
        Contact contact = new Contact(-1, slot,
            new ArmPose(CoralHandlingConstants.reefHeight(slot.level()),
                CoralHandlingConstants.reefPlaceAngle(side)), robot);
        if (atContact(contact)) return contact;
      }
    }
    return null;
  }

  private boolean atContact(Contact contact) {
    // Pivot is finite-range: no wrap-around shortcut when comparing mechanism angles.
    if (!(Math.abs(elevator.getHeight().in(Meters) - contact.arm().height().in(Meters))
            <= SimConstants.CONTACT_HEIGHT_METERS
        && Math.abs(elevator.getPivotAngle().in(Radians) - contact.arm().pivot().in(Radians))
            <= SimConstants.CONTACT_PIVOT_RADIANS
        && Math.abs(elevator.getHeightVelocity().in(MetersPerSecond)) <= SimConstants.CONTACT_LINEAR_SPEED
        && Math.abs(elevator.getPivotVelocity().in(RadiansPerSecond)) <= SimConstants.CONTACT_ANGULAR_SPEED)) {
      return false;
    }
    if (contact.indexerSlot() >= 0) {
      return indexer.getAlignedSlot().orElse(-1) == contact.indexerSlot();
    }
    Pose2d actual = drive.getSimulationPose();
    var speeds = drive.getChassisVelocities();
    return actual.getTranslation().getDistance(contact.robot().getTranslation())
            <= SimConstants.CONTACT_POSITION_METERS
        && Math.abs(actual.getRotation().minus(contact.robot().getRotation()).getRadians())
            <= SimConstants.CONTACT_HEADING_RADIANS
        && Math.hypot(speeds.vx, speeds.vy) <= SimConstants.CONTACT_LINEAR_SPEED
        && Math.abs(speeds.omega) <= SimConstants.CONTACT_ANGULAR_SPEED;
  }

  private CoralState contents(Contact contact) {
    return contact.indexerSlot() >= 0 ? slots[contact.indexerSlot()] : reef.get(contact.reefSlot());
  }

  private void setContents(Contact contact, CoralState color) {
    if (contact.indexerSlot() >= 0) slots[contact.indexerSlot()] = color;
    else reef.put(contact.reefSlot(), color);
  }

  private void updateSensor() {
    updateIndexerSensor();
    boolean connected = coralSensorConnected.get();
    CoralState visible = held;
    if (visible == CoralState.EMPTY) {
      // A released Coral remains visible at contact until the mechanism actually retreats.
      Contact contact = findContact(true);
      if (contact == null) contact = findContact(false);
      if (contact != null) visible = contents(contact);
    }
    gripperColorSensor.setCoralReading(visible, connected);
    gripperRangeSensor.setReading(Meters.of(visible == CoralState.EMPTY
            ? SimConstants.CANRANGE_EMPTY_DISTANCE_METERS : SimConstants.GRIPPER_CANRANGE_PRESENT_DISTANCE_METERS),
        SimConstants.CANRANGE_SIGNAL_STRENGTH,
        connected && gripperRangeConnected.get());
  }

  /** The fixed sensor sees the physical slot, even with a loaded gripper at any arm angle. */
  private void updateIndexerSensor() {
    boolean connected = spinIndexerSensorConnected.get();
    if (InventoryScanConstants.turntableConfigured()) {
      for (int slot = 0; slot < slots.length; slot++) {
        if (Math.abs(MathUtil.inputModulus(indexer.getPosition().in(Rotations)
                - InventoryScanConstants.slotAngle(slot).in(Rotations), -0.5, 0.5))
            <= InventoryScanConstants.ALIGNMENT_TOLERANCE.in(Rotations)) {
          setIndexerSensorReading(slots[slot], connected);
          return;
        }
      }
    }
    setIndexerSensorReading(CoralState.EMPTY, connected);
  }

  private void setIndexerSensorReading(CoralState state, boolean connected) {
    indexerColorSensor.setCoralReading(state, connected);
    indexerRangeSensor.setReading(Meters.of(state == CoralState.EMPTY
            ? SimConstants.CANRANGE_EMPTY_DISTANCE_METERS : SimConstants.INDEXER_CANRANGE_PRESENT_DISTANCE_METERS),
        SimConstants.CANRANGE_SIGNAL_STRENGTH,
        connected && indexerRangeConnected.get());
  }

  private static BooleanEntry booleanSetting(String key, boolean defaultValue) {
    var entry = NetworkTableInstance.getDefault().getBooleanTopic("/" + key).getEntry(defaultValue);
    entry.setDefault(defaultValue);
    return entry;
  }
}
