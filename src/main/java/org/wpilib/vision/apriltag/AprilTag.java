// Copyright (c) 2026 FRC 6907, The G.O.A.T
package org.wpilib.vision.apriltag;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;

/** Binary-compatibility value type for PhotonLib alpha builds using the pre-2027 AprilTag API. */
public final class AprilTag {
  public final int ID;
  public final Pose3d pose;

  public AprilTag(int id, Pose3d pose) {
    ID = id;
    this.pose = Objects.requireNonNull(pose, "pose");
  }
}
