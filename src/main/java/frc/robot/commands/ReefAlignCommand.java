// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ReefAlignConstants.ReefBranch;
import frc.robot.constants.ReefAlignConstants.RobotSide;
import static frc.robot.constants.ReefAlignConstants.angularVelocityToleranceRadPerSec;
import static frc.robot.constants.ReefAlignConstants.headingToleranceRadians;
import static frc.robot.constants.ReefAlignConstants.linearVelocityToleranceMetersPerSec;
import static frc.robot.constants.ReefAlignConstants.maxEntryAngularVelocityRadPerSec;
import static frc.robot.constants.ReefAlignConstants.maxEntryLinearVelocityMetersPerSec;
import static frc.robot.constants.ReefAlignConstants.positionToleranceMeters;
import static frc.robot.constants.ReefAlignConstants.settleTimeSeconds;
import static frc.robot.constants.ReefAlignConstants.targetOffset;
import frc.robot.subsystems.drive.Drive;

/**
 * Local pose correction and hold; owns Drive until the bumper binding cancels it. Target
 * selection and every safety gate stay here; the motion algorithm is seafoam green's
 * DriveToPose, driven as an embedded controller with a manual lifecycle.
 */
public final class ReefAlignCommand implements Command {
  public record AlignmentTarget(Pose2d pose, ReefBranch branch, RobotSide side) {}
  public record ReefSelection(int tagId, Pose2d pose, RobotSide side) {}
  private final Consumer<AlignmentTarget> targetObserver;
  private final Integer fixedTagId;
  private final RobotSide fixedSide;
  private final Drive drive;
  private final ReefBranch branch;
  private final BooleanSupplier visionFresh;
  private final Alert unavailableAlert;

  private DriveToPose controller; // the DriveToPose algorithm, run manually
  private Pose2d tagPose;
  private Pose2d targetPose;
  private RobotSide side;
  private double settledSince = Double.NaN;
  private boolean reachedGoal;
  private boolean blocked;
  private boolean visionPaused;

  public ReefAlignCommand(Drive drive, ReefBranch branch, BooleanSupplier visionFresh) {
    this(drive, branch, visionFresh, target -> {});
  }

  /** Reports null on a new/failed/paused alignment, and the locked target after settling at it. */
  public ReefAlignCommand(Drive drive, ReefBranch branch, BooleanSupplier visionFresh,
      Consumer<AlignmentTarget> targetObserver) {
    this(drive, branch, visionFresh, targetObserver, null, null);
  }

  /** Fixed-face/side variant for auto; null keeps the existing nearest-face/heading selection. */
  public ReefAlignCommand(Drive drive, ReefBranch branch, BooleanSupplier visionFresh,
      Consumer<AlignmentTarget> targetObserver, Integer fixedTagId, RobotSide fixedSide) {
    this.fixedTagId = fixedTagId;
    this.fixedSide = fixedSide;
    this.targetObserver = targetObserver;
    this.drive = drive;
    this.branch = branch;
    this.visionFresh = visionFresh;
    unavailableAlert =
      new Alert(
        "Reef alignment " + branch + " unavailable.",
        "Reef alignment is unavailable.",
        Level.MEDIUM);
  }

  @Override
  public void run(Coroutine coroutine) {
    initialize();
    while (true) {
      execute();
      coroutine.yield();
    }
  }

  @Override
  public void onCancel() {
    end(true);
  }

  @Override
  public String name() {
    return "ReefAlign:" + branch;
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }

  public void initialize() {
    targetObserver.accept(null);
    drive.stop();
    blocked = false;
    visionPaused = false;
    reachedGoal = false;
    settledSince = Double.NaN;
    controller = null;
    targetPose = null;
    tagPose = null;
    unavailableAlert.set(false);
    Logger.recordOutput("ReefAlign/AtGoal", false);
    Logger.recordOutput("ReefAlign/Branch", branch.name());
    Logger.recordOutput("ReefAlign/Target", new Pose2d[0]);
    Logger.recordOutput("ReefAlign/TagId", -1);
    Logger.recordOutput("ReefAlign/RobotSide", "UNSELECTED");
    Logger.recordOutput("ReefAlign/RequestedFieldSpeeds", new ChassisVelocities());

    
    Pose2d current = drive.getPose();
    
    var selection = (fixedTagId == null ? selectReef(current) : selectReef(current, fixedTagId))
        .orElse(null);
    if (selection == null) {
      block("NO_REEF_FACE");
      return;
    }
    tagPose = selection.pose();
    side = fixedSide == null ? selection.side() : fixedSide;
    Logger.recordOutput("ReefAlign/TagId", selection.tagId());
    Logger.recordOutput("ReefAlign/RobotSide", side.name());
    targetPose = tagPose.transformBy(new Transform2d(targetOffset(side, branch),
        Rotation2d.fromRadians(side == RobotSide.LEFT ? Math.PI / 2 : -Math.PI / 2)));
    Logger.recordOutput("ReefAlign/Target", new Pose2d[] {targetPose});
    ChassisVelocities measured = drive.getChassisVelocities();
    if (Math.hypot(measured.vx, measured.vy)
        > maxEntryLinearVelocityMetersPerSec
      || Math.abs(measured.omega) > maxEntryAngularVelocityRadPerSec) {
      block("ENTRY_SPEED_TOO_HIGH_OR_INVALID");
      return;
    }
    startController();
    Logger.recordOutput("ReefAlign/Status", "ALIGNING");
  }

  public void execute() {
    if (blocked) {
      // Keep the requirement while held: a failure must never reactivate the joystick command.
      drive.stop();
      return;
    }
    Pose2d current = drive.getPose();
    if (!visionFresh.getAsBoolean()) {
      // Retain Drive and the locked target while waiting; no joystick handoff or fault latch.
      visionPaused = true;
      settledSince = Double.NaN;
      targetObserver.accept(null);
      drive.stop();
      Logger.recordOutput("ReefAlign/Status", "WAITING_FOR_VISION");
      Logger.recordOutput("ReefAlign/AtGoal", false);
      Logger.recordOutput("ReefAlign/RequestedFieldSpeeds", new ChassisVelocities());
      return;
    }
    if (visionPaused) {
      startController(); // vision resumed: re-profile from wherever the robot now sits
      visionPaused = false;
    }
    // The DriveToPose algorithm: polar drive/theta profiles with a near-goal feedforward scaler.
    controller.execute();
    ChassisVelocities measured = drive.getChassisVelocities();
    double distance = current.getTranslation().getDistance(targetPose.getTranslation());
    double headingError = Math.abs(targetPose.getRotation().minus(current.getRotation()).getRadians());
    Logger.recordOutput("ReefAlign/PositionErrorMeters", distance);
    Logger.recordOutput("ReefAlign/HeadingErrorRadians", headingError);
    boolean withinTolerance = distance <= positionToleranceMeters
        && headingError <= headingToleranceRadians
        && Math.hypot(measured.vx, measured.vy)
            <= linearVelocityToleranceMetersPerSec
        && Math.abs(measured.omega) <= angularVelocityToleranceRadPerSec;
    boolean controllerAtGoal = controller.atGoal();
    if (withinTolerance && controllerAtGoal) {
      if (!Double.isFinite(settledSince)) settledSince = Timer.getTimestamp();
    } else {
      settledSince = Double.NaN;
    }
    boolean atGoal = Double.isFinite(settledSince)
        && Timer.getTimestamp() - settledSince >= settleTimeSeconds;
    reachedGoal |= atGoal;
    if (atGoal) targetObserver.accept(new AlignmentTarget(targetPose, branch, side));
    Logger.recordOutput("ReefAlign/AtGoal", atGoal);
    Logger.recordOutput("ReefAlign/Status", atGoal ? "HOLDING" : "ALIGNING");
  }

  /** Auto can terminate on arrival or rejection; held bumper commands still do not finish. */
  public boolean hasSucceeded() { return reachedGoal && !visionPaused && !blocked; }
  public boolean hasFailed() { return blocked; }

  public void end(boolean interrupted) {
    if (controller != null) {
      controller.end(interrupted); // stops the drive and clears the DriveToPose logs
    } else {
      drive.stop();
    }
    unavailableAlert.set(false);
    Logger.recordOutput("ReefAlign/AtGoal", false);
    Logger.recordOutput("ReefAlign/Status", "IDLE");
    Logger.recordOutput("ReefAlign/RequestedFieldSpeeds", new ChassisVelocities());
  }

  /** Shared by bumper alignment and B/X/Y preparation; equal heading corrections choose RIGHT. */
  public static Optional<ReefSelection> selectReef(Pose2d current) {
    Pose2d nearest = null;
    int tagId = -1;
    double closestDistance = Double.POSITIVE_INFINITY;
    // Ascending IDs resolve ties among this alliance's six faces, independent of visible tag order.
    for (int id : FieldConstants.reefTagIds().stream().sorted().toList()) {
      var tag = FieldConstants.aprilTagLayout.getTagPose(id);
      if (tag.isEmpty()) continue;
      Pose2d candidate = tag.get().toPose2d();
      double distance = current.getTranslation().getDistance(candidate.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        nearest = candidate;
        tagId = id;
      }
    }
    if (nearest == null) return Optional.empty();
    return selectReef(current, tagId);
  }

  /** Uses one explicit allied Reef face, never substitutes a nearer face for an invalid tag ID. */
  public static Optional<ReefSelection> selectReef(Pose2d current, int tagId) {
    if (!FieldConstants.reefTagIds().contains(tagId)) return Optional.empty();
    var tag = FieldConstants.aprilTagLayout.getTagPose(tagId);
    if (tag.isEmpty()) return Optional.empty();
    Pose2d nearest = tag.get().toPose2d();
    Rotation2d leftHeading = nearest.getRotation().plus(Rotation2d.fromRadians(Math.PI / 2));
    Rotation2d rightHeading = nearest.getRotation().minus(Rotation2d.fromRadians(Math.PI / 2));
    RobotSide selectedSide = Math.abs(leftHeading.minus(current.getRotation()).getRadians())
            < Math.abs(rightHeading.minus(current.getRotation()).getRadians())
        ? RobotSide.LEFT : RobotSide.RIGHT;
    return Optional.of(new ReefSelection(tagId, nearest, selectedSide));
  }

 

  /** Creates the DriveToPose controller fresh, profiling from the robot's current pose and speed. */
  private void startController() {
    controller = new DriveToPose(drive, () -> targetPose);
    controller.initialize();
  }


  private void block(String reason) {
    targetObserver.accept(null);
    blocked = true;
    drive.stop();
    unavailableAlert.setText("Reef alignment " + branch + " unavailable: " + reason);
    unavailableAlert.set(true);
    Logger.recordOutput("ReefAlign/Status", reason);
    Logger.recordOutput("ReefAlign/AtGoal", false);
    Logger.recordOutput("ReefAlign/RequestedFieldSpeeds", new ChassisVelocities());
  }

  
}
