// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import frc.robot.subsystems.spinindexer.SpinIndexerConstants;

/** Scan geometry uses the same elevator HOME and turntable encoder reference as normal transfers. */
public final class InventoryScanConstants {
  public static final Angle SLOT_ANGLE_OFFSET = Degrees.of(Double.NaN); // TODO: Signed turntable angle from slot transfer alignment to the shared fixed CANrange/REV color observation position, deg; positive follows encoder direction and does not depend on arm pose. Both sensors must observe the same slot at this offset.
  public static final Angle ALIGNMENT_TOLERANCE = Degrees.of(2.0); // TODO: Maximum scan-stop angle error, deg; verify the sensor sees only the selected slot throughout this window.
  public static final double SETTLE_SECONDS = Double.NaN; // TODO: Continuous time within scan angle and existing slot transfer-speed limits before reading, s.
  public static final double SENSOR_FLUSH_SECONDS = Double.NaN; // TODO: Wait after reaching each slot or gripper observation pose before discarding old detection, s; shared by manual inventory scan and automatic gripper recovery. Cover the 100 ms measurement integration and actual sensor latency.

  public static Angle slotAngle(int slot) {
    Angle transferAngle = switch (slot) {
      case 0 -> SpinIndexerConstants.SLOT_1_ALIGNMENT_ANGLE;
      case 1 -> SpinIndexerConstants.SLOT_2_ALIGNMENT_ANGLE;
      case 2 -> SpinIndexerConstants.SLOT_3_ALIGNMENT_ANGLE;
      default -> throw new IndexOutOfBoundsException("Scan slot must be 0, 1, or 2: " + slot);
    };
    return Rotations.of(transferAngle.in(Rotations) + SLOT_ANGLE_OFFSET.in(Rotations));
  }

  public static boolean turntableConfigured() {
    double tolerance = ALIGNMENT_TOLERANCE.in(Rotations);
    if (!Double.isFinite(SLOT_ANGLE_OFFSET.in(Rotations))
        || !CoralHandlingConstants.positive(tolerance) || tolerance >= 0.125
        || !CoralHandlingConstants.positive(SETTLE_SECONDS)) return false;
    for (int slot = 0; slot < SpinIndexerConstants.SLOT_COUNT; slot++) {
      if (!Double.isFinite(slotAngle(slot).in(Rotations))) return false;
      for (int other = 0; other < slot; other++) {
        if (Math.abs(MathUtil.inputModulus(
                slotAngle(slot).in(Rotations) - slotAngle(other).in(Rotations), -0.5, 0.5))
            <= 2 * tolerance) return false;
      }
    }
    return true;
  }

  private InventoryScanConstants() {}
}
