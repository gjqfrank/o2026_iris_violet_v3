// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.units.measure.Distance;
import frc.robot.util.AllianceFlipUtil;
import java.util.Set;
import java.util.stream.Stream;

/**
 * Contains information for the 2025 REEFSCAPE field.
 *
 * <p>All coordinates use the WPILib field coordinate system
 * with the origin on the blue alliance side.
 */
public final class FieldConstants {

  // TODO: Verify the actual GOATSCAPE tag positions/orientations and field dimensions match this Welded layout.
  public static final Field kFieldTagLayout = Field.loadField(Fields.FRC_2025_REEFSCAPE_WELDED);

  public static final Field aprilTagLayout = kFieldTagLayout;

  public static final Distance fieldLength = Meters.of(kFieldTagLayout.getFieldLength());
  public static final Distance fieldWidth = Meters.of(kFieldTagLayout.getFieldWidth());

  public static final Set<Integer> BLUE_REEF_TAG_IDS = Set.of(17, 18, 19, 20, 21, 22);
  public static final Set<Integer> RED_REEF_TAG_IDS = Set.of(6, 7, 8, 9, 10, 11);
  public static final int BLUE_BACK_SIDE_TAG_ID = 20;
  public static final int RED_BACK_SIDE_TAG_ID = 11;
  public static final Set<Integer> ALL_REEF_TAG_IDS = Set.copyOf(Stream.concat(
      BLUE_REEF_TAG_IDS.stream(), RED_REEF_TAG_IDS.stream()).toList());

  /** DS alliance chooses the Reef; poses always remain in the blue-origin frame. */
  public static Set<Integer> reefTagIds() {
    return AllianceFlipUtil.shouldFlip() ? RED_REEF_TAG_IDS : BLUE_REEF_TAG_IDS;
  }

  private FieldConstants() {}
}
