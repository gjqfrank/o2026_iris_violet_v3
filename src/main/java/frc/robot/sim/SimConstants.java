// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.sim;


import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;

/** Assumed SIM geometry and contact tolerances; never configures REAL hardware. */
public final class SimConstants {
  // TODO: Replace assumed SIM mounts and camera properties with the installed camera specifications when available.
  public static final List<CameraConfig> CAMERA_CONFIGS = List.of(
      new CameraConfig("SimFront", mount(0.25, 0, 0), 1.0),
      new CameraConfig("SimLeft", mount(0, 0.25, Math.PI / 2), 1.0),
      new CameraConfig("SimRight", mount(0, -0.25, -Math.PI / 2), 1.0));
    public static final AprilTagFieldLayout REEF_LAYOUT = new AprilTagFieldLayout(
      VisionConstants.aprilTagLayout.getTags().stream()
        .filter(tag -> FieldConstants.ALL_REEF_TAG_IDS.contains(tag.getID()))
        .map(tag -> new AprilTag(tag.getID(), tag.getPose())).toList(),
      VisionConstants.aprilTagLayout.getFieldLength(),
      VisionConstants.aprilTagLayout.getFieldWidth());
  public static final int CAMERA_WIDTH = 640;
  public static final int CAMERA_HEIGHT = 480;
  public static final double CAMERA_DIAGONAL_FOV_DEGREES = 90;
  public static final double CAMERA_FPS = 30;
  public static final double CAMERA_LATENCY_MS = 35;
  public static final double CAMERA_LATENCY_STDDEV_MS = 5;
  public static final double CAMERA_PIXEL_ERROR = 0.25;
  public static final double CAMERA_PIXEL_ERROR_STDDEV = 0.08;
  // TODO: Adjust these SIM contact tolerances and cylinder travel time to match the modeled mechanism.
  public static final double CONTACT_HEIGHT_METERS = 0.015;
  public static final double CONTACT_PIVOT_RADIANS = Math.toRadians(3);
  public static final double CONTACT_POSITION_METERS = 0.04;
  public static final double CONTACT_HEADING_RADIANS = Math.toRadians(3);
  public static final double CONTACT_LINEAR_SPEED = 0.05;
  public static final double CONTACT_ANGULAR_SPEED = 0.1;
  public static final double JAW_ACTUATION_SECONDS = 0.08;
  // TODO: Replace assumed slot optical readings with measurements at the fixed REV sensor mount.
  public static final int INDEXER_SCAN_WHITE_CHANNEL = 131071;
  // TODO: Replace assumed SIM distances/signal strength with readings from the installed CANranges.
  public static final double GRIPPER_CANRANGE_PRESENT_DISTANCE_METERS = 0.05;
  public static final double INDEXER_CANRANGE_PRESENT_DISTANCE_METERS = 0.10;
  public static final double CANRANGE_EMPTY_DISTANCE_METERS = 0.50;
  public static final double CANRANGE_SIGNAL_STRENGTH = 10000;

  private static Transform3d mount(double x, double y, double yaw) {
    // Negative pitch points optical +X upward in WPILib coordinates.
    return new Transform3d(new Translation3d(x, y, 0.55),
        new Rotation3d(0, Math.toRadians(-15), yaw));
  }

  private SimConstants() {}
}
