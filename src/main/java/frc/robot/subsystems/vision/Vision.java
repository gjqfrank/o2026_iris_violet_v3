// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.system.Timer;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Filters three cameras in the fixed field frame and forwards measurements to Drive, never motors. */
public class Vision implements Mechanism {
  private final VisionConsumer consumer;
  private final List<CameraConfig> cameraConfigs;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Alert[] unconfiguredAlerts;
  private final boolean[] cameraConfigurationValid;
  private final double[] lastProcessedTimestamp;
  private double lastForwardedTimestamp = Double.NEGATIVE_INFINITY;
  private Set<Integer> activeTagIds = allowedTagIds();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, VisionConstants.cameraConfigs, io);
  }

  public Vision(VisionConsumer consumer, List<CameraConfig> configs, VisionIO... io) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    cameraConfigs = List.copyOf(configs);
    if (io.length != cameraConfigs.size()) {
      throw new IllegalArgumentException("Vision IO order must match all three camera configurations");
    }
    this.consumer = consumer;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];
    unconfiguredAlerts = new Alert[io.length];
    cameraConfigurationValid = new boolean[io.length];
    lastProcessedTimestamp = new double[io.length];
    Arrays.fill(lastProcessedTimestamp, Double.NEGATIVE_INFINITY);
    for (int i = 0; i < io.length; i++) {
      cameraConfigurationValid[i] = Constants.getMode() != Mode.REAL || REAL_IO_CONFIGURED;
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert("camera diaconnected","Vision camera " + (i + 1) + " disconnected.", Level.MEDIUM);
      unconfiguredAlerts[i] =
          new Alert("camera unconfigured","Vision camera " + (i + 1) + " name/extrinsics not configured.",
              Level.MEDIUM);
    }
  }

  private void periodic() {
    Set<Integer> selectedTagIds = allowedTagIds();
    if (!activeTagIds.equals(selectedTagIds)) {
      activeTagIds = selectedTagIds;
      lastForwardedTimestamp = Double.NEGATIVE_INFINITY;
    }
    double now = Timer.getTimestamp();
    List<PendingObservation> pending = new ArrayList<>();
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      boolean configured = cameraConfigurationValid[i];
      unconfiguredAlerts[i].set(!configured);
      disconnectedAlerts[i].set(
          configured && !inputs[i].connected && Constants.getMode() != Mode.SIM);
      Logger.recordOutput("Vision/Camera" + i + "/Configured", configured);
      for (PoseObservation observation : inputs[i].poseObservations) {
        pending.add(new PendingObservation(i, observation));
      }
    }
    // Camera index does not define arrival order. Fuse this cycle's observations by capture time.
    pending.sort(Comparator.comparingDouble(item -> item.observation().timestamp()));
    List<Pose3d> accepted = new ArrayList<>();
    List<Pose3d> rejected = new ArrayList<>();
    List<String> reasons = new ArrayList<>();
    int[] forwardedCounts = new int[io.length];
    for (PendingObservation item : pending) {
      int index = item.cameraIndex();
      PoseObservation observation = item.observation();
      String reason = rejectionReason(index, observation, now);
      if (reason.isEmpty() && observation.timestamp() <= lastProcessedTimestamp[index]) {
        reason = "DUPLICATE_OR_OLD_FRAME";
      }
      // Do not advance the watermark with a future or unsynchronised timestamp.
      if (Double.isFinite(observation.timestamp()) && observation.timestamp() > 0
          && observation.timestamp() <= now + maxFutureTimestampSecs
          && observation.timeSyncValid()) {
        lastProcessedTimestamp[index] =
            Math.max(lastProcessedTimestamp[index], observation.timestamp());
      }
      if (reason.isEmpty() && observation.timestamp() < lastForwardedTimestamp) {
        // WPILib discards later corrections when an older correction is inserted.
        reason = "OLDER_THAN_LAST_FORWARDED";
      }
      if (!reason.isEmpty()) {
        if (observation.pose() != null) rejected.add(observation.pose());
        reasons.add("Cam" + index + ":" + reason);
        continue;
      }
      boolean updateHeading = observation.type() == PoseObservationType.PHOTONVISION_MULTITAG;
      double factor = Math.pow(observation.averageTagDistance(), 2) / observation.tagCount()
          * cameraConfigs.get(index).stdDevFactor();
      if (!Double.isFinite(factor)) {
        rejected.add(observation.pose());
        reasons.add("Cam" + index + ":INVALID_UNCERTAINTY");
        continue;
      }
      double linearStdDev = Math.max(minLinearStdDev,
          linearStdDevBaseline * factor * (updateHeading ? linearStdDevPhotonMultitagFactor : 1));
      double angularStdDev = updateHeading
          ? Math.max(minAngularStdDev, angularStdDevBaseline * factor)
          : Double.POSITIVE_INFINITY;
      consumer.accept(observation.pose().toPose2d(), observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev), updateHeading);
      lastForwardedTimestamp = observation.timestamp();
      accepted.add(observation.pose());
      forwardedCounts[index]++;
      Logger.recordOutput("Vision/Camera" + index + "/LastForwardedTimestamp",
          observation.timestamp());
      Logger.recordOutput("Vision/Camera" + index + "/LastForwardedPose", observation.pose());
      Logger.recordOutput("Vision/Camera" + index + "/LastStdDevs",
          new double[] {linearStdDev, linearStdDev, angularStdDev});
    }
    for (int i = 0; i < io.length; i++) {
      Logger.recordOutput("Vision/Camera" + i + "/ForwardedCount", forwardedCounts[i]);
    }
    // Forwarded means submitted to Drive; it does not assert that estimator history was available.
    Logger.recordOutput("Vision/Summary/RobotPosesForwarded", accepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPosesRejected", rejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RejectReasons", reasons.toArray(new String[0]));
  }

  /** Recent filtered pose submission, not raw target visibility or a guarantee of estimator accuracy. */
  public boolean hasRecentPoseMeasurement(double maxAgeSeconds) {
    double age = Timer.getTimestamp() - lastForwardedTimestamp;
    return Double.isFinite(maxAgeSeconds) && maxAgeSeconds > 0
        && Double.isFinite(age) && age >= -maxFutureTimestampSecs && age <= maxAgeSeconds;
  }

  private String rejectionReason(int index, PoseObservation observation, double now) {
    if (!cameraConfigurationValid[index]) return "NOT_CONFIGURED";
    if (!inputs[index].connected) return "DISCONNECTED";
    if (!observation.timeSyncValid()) return "TIME_SYNC_UNAVAILABLE";
    Pose3d pose = observation.pose();
    if (pose == null || !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getZ()) || !Double.isFinite(pose.getRotation().getX())
        || !Double.isFinite(pose.getRotation().getY())
        || !Double.isFinite(pose.getRotation().getZ())
        || !Double.isFinite(observation.timestamp())
        || !Double.isFinite(observation.averageTagDistance())
        || observation.averageTagDistance() <= 0
        || observation.type() == null) return "INVALID_DATA";
    if (observation.timestamp() <= 0 || now - observation.timestamp() > maxObservationDelaySecs) {
      return "TOO_OLD";
    }
    if (observation.timestamp() - now > maxFutureTimestampSecs) return "FUTURE_TIMESTAMP";
    boolean single = observation.type() == PoseObservationType.PHOTONVISION_SINGLETAG;
    if (single && (!Double.isFinite(observation.ambiguity()) || observation.ambiguity() < 0
        || observation.ambiguity() > maxSingleTagAmbiguity)) return "AMBIGUITY";
    if (single && (!Double.isFinite(observation.yaw())
        || !Double.isFinite(observation.pitch()) || !Double.isFinite(observation.area()))) {
      return "INVALID_TARGET_DATA";
    }
    if (single && Math.abs(observation.yaw()) > maxPhotonYawDeg) return "TARGET_YAW";
    if (single && Math.abs(observation.pitch()) > maxPhotonPitchDeg) return "TARGET_PITCH";
    if (single && observation.area() < minPhotonArea) return "TARGET_AREA";
    if (Math.abs(pose.getZ()) > maxZError) return "ROBOT_Z";
    if (pose.getX() < 0 || pose.getX() > aprilTagLayout.getFieldLength()
        || pose.getY() < 0 || pose.getY() > aprilTagLayout.getFieldWidth()) {
      return "OUT_OF_FIELD";
    }
    double cameraYaw = pose.plus(cameraConfigs.get(index).robotToCamera()).getRotation().getZ();
    for (int id : observation.tagIds()) {
      if (!activeTagIds.contains(id)) return "OTHER_ALLIANCE_TAG";
      var tagPose = aprilTagLayout.getTagPose(id);
      if (tagPose.isEmpty()) return "UNKNOWN_TAG";
      if (Math.cos(cameraYaw - tagPose.get().getRotation().getZ()) > 0) return "TAG_FACING";
    }
    return "";
  }

  private record PendingObservation(int cameraIndex, PoseObservation observation) {}

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs, boolean updateHeading);
  }
}
