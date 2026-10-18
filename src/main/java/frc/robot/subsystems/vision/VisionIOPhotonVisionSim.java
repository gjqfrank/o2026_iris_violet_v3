// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.math.geometry.Rotation2d;
import frc.robot.sim.SimConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;

/** Simulated PhotonVision camera using the same result reader and robot-pose filtering as real IO. */
public final class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static final Map<VisionSystemSim, Boolean> WORLDS_WITH_REEF_TAGS =
      Collections.synchronizedMap(new WeakHashMap<>());

  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonVisionSim(CameraConfig config, VisionSystemSim world) {
    this(config, world, new PhotonCamera(config.name()));
  }

  private VisionIOPhotonVisionSim(CameraConfig config, VisionSystemSim world, PhotonCamera camera) {
    super(config, camera);
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(
        SimConstants.CAMERA_WIDTH,
        SimConstants.CAMERA_HEIGHT,
        Rotation2d.fromDegrees(SimConstants.CAMERA_DIAGONAL_FOV_DEGREES));
    properties.setCalibError(
        SimConstants.CAMERA_PIXEL_ERROR, SimConstants.CAMERA_PIXEL_ERROR_STDDEV);
    properties.setFPS(SimConstants.CAMERA_FPS);
    properties.setAvgLatencyMs(SimConstants.CAMERA_LATENCY_MS);
    properties.setLatencyStdDevMs(SimConstants.CAMERA_LATENCY_STDDEV_MS);

    cameraSim = new PhotonCameraSim(camera, properties, SimConstants.REEF_LAYOUT);
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableDrawWireframe(false);
    synchronized (WORLDS_WITH_REEF_TAGS) {
      if (!WORLDS_WITH_REEF_TAGS.containsKey(world)) {
        world.addAprilTags(SimConstants.REEF_LAYOUT);
        WORLDS_WITH_REEF_TAGS.put(world, Boolean.TRUE);
      }
    }
    world.addCamera(cameraSim, config.robotToCamera());
  }
}
