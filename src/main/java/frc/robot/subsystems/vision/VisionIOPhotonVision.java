// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Converts calibrated PhotonVision camera poses into robot poses; owns the sole camera reader. */
public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;

  public VisionIOPhotonVision(CameraConfig config) {
    this(config, new PhotonCamera(config.name()));
  }

  protected VisionIOPhotonVision(CameraConfig config, PhotonCamera camera) {
    this.camera = camera;
    robotToCamera = config.robotToCamera();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PoseObservation> observations = new ArrayList<>();
    // Reading exactly once drains PhotonVision's FIFO without duplicating old frames.
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      boolean timeSyncValid = result.metadata.timeSinceLastPong >= 0
          && result.metadata.timeSinceLastPong <= maxTimeSinceLastPongSecs * 1e6;
      PoseObservation observation = multitagObservation(result, timeSyncValid);
      if (observation == null) observation = singleTagObservation(result, timeSyncValid);
      if (observation != null) observations.add(observation);
    }
    observations.sort(Comparator.comparingDouble(PoseObservation::timestamp));
    int first = Math.max(0, observations.size() - maxPoseObservationsPerUpdate);
    inputs.poseObservations =
        observations.subList(first, observations.size()).toArray(new PoseObservation[0]);
    inputs.tagIds = observations.subList(first, observations.size()).stream()
        .flatMapToInt(observation -> java.util.Arrays.stream(observation.tagIds()))
        .distinct().sorted().toArray();
  }

  private PoseObservation multitagObservation(PhotonPipelineResult result, boolean timeSyncValid) {
    if (result.getMultiTagResult().isEmpty()) return null;
    var multitag = result.getMultiTagResult().get();
    int[] ids = multitag.fiducialIDsUsed.stream().mapToInt(Short::intValue).toArray();
    if (ids.length < 2 || java.util.Arrays.stream(ids).distinct().count() != ids.length
        || java.util.Arrays.stream(ids).anyMatch(id -> !allowedTagIds().contains(id))) {
      // Removing IDs after a PNP solve cannot remove their influence. Fall back to an allowed tag.
      return null;
    }
    double distanceSum = 0;
    for (int id : ids) {
      PhotonTrackedTarget target = result.getTargets().stream()
          .filter(candidate -> candidate.getFiducialId() == id).findFirst().orElse(null);
      if (target == null) return null;
      distanceSum += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    Pose3d robotPose = Pose3d.ZERO.plus(multitag.estimatedPose.best)
        .relativeTo(aprilTagLayout.getOrigin()).plus(robotToCamera.inverse());
    return new PoseObservation(result.getTimestampSeconds(), robotPose,
        multitag.estimatedPose.ambiguity, ids.length, distanceSum / ids.length,
        0, 0, 1, ids, timeSyncValid, PoseObservationType.PHOTONVISION_MULTITAG);
  }

  private PoseObservation singleTagObservation(PhotonPipelineResult result, boolean timeSyncValid) {
    PhotonTrackedTarget target = result.getTargets().stream()
        .filter(candidate -> allowedTagIds().contains(candidate.getFiducialId())
            && Double.isFinite(candidate.getPoseAmbiguity())
            && candidate.getPoseAmbiguity() >= 0)
        .min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity)).orElse(null);
    if (target == null) return null;
    var tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
    if (tagPose.isEmpty()) return null;
    Pose3d robotPose = tagPose.get().plus(target.getBestCameraToTarget().inverse())
        .plus(robotToCamera.inverse());
    return new PoseObservation(result.getTimestampSeconds(), robotPose,
        target.getPoseAmbiguity(), 1, target.getBestCameraToTarget().getTranslation().getNorm(),
        target.getYaw(), target.getPitch(), target.getArea(),
        new int[] {target.getFiducialId()}, timeSyncValid, PoseObservationType.PHOTONVISION_SINGLETAG);
  }
}
