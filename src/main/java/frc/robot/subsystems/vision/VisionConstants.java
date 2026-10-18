// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;


import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Transform3d;
import frc.robot.constants.FieldConstants;

import java.util.List;
import java.util.Set;

public final class VisionConstants {
  public static final boolean REAL_IO_CONFIGURED = false; // TODO: Enable after configuring all camera names, extrinsics and uncertainty factors.
  // Fixed blue-origin frame, independent of alliance. PhotonVision must use the same Welded layout.
  public static final Field aprilTagLayout = FieldConstants.aprilTagLayout;
  public static Set<Integer> allowedTagIds() {
    return FieldConstants.reefTagIds();
  }

  /** Translation in meters: +X forward, +Y left, +Z up; rotation in radians. */
  public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {}

  public static final List<CameraConfig> cameraConfigs =
      // TODO: Verify each camera's PhotonVision intrinsics/calibration matches its actual lens and resolution.
      List.of(
          // TODO: Camera 1 PhotonVision name, x/y/z (m), roll/pitch/yaw (rad); validate stdDevFactor.
          new CameraConfig("", null, 1.0),
          // TODO: Camera 2 PhotonVision name, x/y/z (m), roll/pitch/yaw (rad); validate stdDevFactor.
          new CameraConfig("", null, 1.0),
          // TODO: Camera 3 PhotonVision name, x/y/z (m), roll/pitch/yaw (rad); validate stdDevFactor.
          new CameraConfig("", null, 1.0));

  // Starting thresholds from glacier_blue; validate with the installed cameras.
  public static final int maxPoseObservationsPerUpdate = 5; // TODO: Validate frame-burst processing budget with all three cameras active.
  public static final double maxSingleTagAmbiguity = 0.2; // TODO: Validate single-tag ambiguity rejection using camera logs.
  public static final double maxPhotonYawDeg = 160.0; // TODO: Validate target yaw range with installed camera FOV, deg.
  public static final double maxPhotonPitchDeg = 160.0; // TODO: Validate target pitch range with installed camera FOV, deg.
  public static final double minPhotonArea = 0.2; // TODO: Validate minimum target area; percent of image, not a fraction.
  public static final double maxObservationDelaySecs = 0.5; // TODO: Validate maximum usable image age with measured latency, s.
  public static final double maxFutureTimestampSecs = 0.02; // TODO: Validate timestamp lead allowance with clock synchronization, s.
  public static final double maxTimeSinceLastPongSecs = 5.0; // TODO: Validate time-sync freshness timeout, s.
  public static final double maxZError = 0.25; // TODO: Validate robot-origin height rejection, m.
  public static final double linearStdDevBaseline = 0.02; // TODO: Fit position uncertainty from logs, m at 1 m with one tag.
  public static final double angularStdDevBaseline = 0.06; // TODO: Fit heading uncertainty from logs, rad at 1 m with one tag.
  public static final double linearStdDevPhotonMultitagFactor = 0.5; // TODO: Validate multi-tag position uncertainty multiplier.
  public static final double minLinearStdDev = 0.01; // TODO: Validate position uncertainty floor to avoid excessive trust, m.
  public static final double minAngularStdDev = 0.02; // TODO: Validate heading uncertainty floor, rad.

  private VisionConstants() {}
}
