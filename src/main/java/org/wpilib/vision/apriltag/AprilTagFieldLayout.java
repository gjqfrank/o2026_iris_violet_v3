// Copyright (c) 2026 FRC 6907, The G.O.A.T
package org.wpilib.vision.apriltag;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;

/**
 * Adapter for PhotonLib builds compiled against the WPILib AprilTagFieldLayout API removed in 2027.
 * The robot code uses {@link Field}; this class only bridges PhotonLib's simulation/PNP signatures.
 */
public final class AprilTagFieldLayout {
  private final List<AprilTag> tags;
  private final double fieldLength;
  private final double fieldWidth;

  public AprilTagFieldLayout(List<AprilTag> tags, double fieldLength, double fieldWidth) {
    this.tags = List.copyOf(tags);
    this.fieldLength = fieldLength;
    this.fieldWidth = fieldWidth;
  }

  /** Loads the matching built-in 2025 field for PhotonLib's legacy default constructor path. */
  public static AprilTagFieldLayout loadField(AprilTagFields ignored) {
    Field field = Field.loadField(Fields.FRC_2025_REEFSCAPE_WELDED);
    return new AprilTagFieldLayout(
        field.getTags().stream()
            .map(tag -> new AprilTag(tag.getID(), tag.getPose()))
            .collect(Collectors.toList()),
        field.getFieldLength(),
        field.getFieldWidth());
  }

  public List<AprilTag> getTags() {
    return tags;
  }

  public Optional<Pose3d> getTagPose(int id) {
    return tags.stream().filter(tag -> tag.ID == id).map(tag -> tag.pose).findFirst();
  }

  public double getFieldLength() {
    return fieldLength;
  }

  public double getFieldWidth() {
    return fieldWidth;
  }
}
