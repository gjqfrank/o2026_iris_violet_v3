// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /**
   * Timestamp is image capture time in FPGA seconds, pose is the robot origin in field coordinates.
   * IO supplies unique allowed Reef tag IDs, tagCount equal to their count, and a matching type
   * (one tag for SingleTag, at least two for MultiTag).
   */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double yaw,
      double pitch,
      double area,
      int[] tagIds,
      boolean timeSyncValid,
      PoseObservationType type) {}

  enum PoseObservationType {
    PHOTONVISION_MULTITAG,
    PHOTONVISION_SINGLETAG
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
