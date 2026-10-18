// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.util;

import static frc.robot.subsystems.spinindexer.CoralState.BLACK;
import static frc.robot.subsystems.spinindexer.CoralState.WHITE;

import frc.robot.constants.ReefAlignConstants.ReefBranch;
import frc.robot.subsystems.spinindexer.CoralState;
import java.util.Objects;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringEntry;

/** Shared scoring colors for all Reef faces on either alliance; does not track branch occupancy. */
public final class ReefTargetColors {
  public enum Level { L4, L3, L2 }

  private final StringEntry chooser =
      NetworkTableInstance.getDefault().getStringTopic("/Reef/TargetColors").getEntry("UNCONFIGURED");

  public ReefTargetColors() {
    chooser.setDefault("UNCONFIGURED");
  }

  /** LEFT/RIGHT is viewed facing the Reef face, independent of the robot's scoring side. */
  public Optional<CoralState> getTargetColor(Level level, ReefBranch branch) {
    Objects.requireNonNull(level, "level");
    Objects.requireNonNull(branch, "branch");
    Pattern pattern = selectedPattern();
    if (pattern == Pattern.UNCONFIGURED) return Optional.empty();
    CoralState left = pattern.leftColor(level);
    return Optional.of(branch == ReefBranch.LEFT ? left : opposite(left));
  }

  public void periodic() {
    Pattern pattern = selectedPattern();
    boolean configured = pattern != Pattern.UNCONFIGURED;
    Logger.recordOutput("Reef/Targets/Configured", configured);
    Logger.recordOutput("Reef/Targets/Pattern", pattern.label());
    for (Level level : Level.values()) {
      CoralState left = pattern.leftColor(level);
      String prefix = "Reef/Targets/" + level.name();
      Logger.recordOutput(prefix + "/LEFT", configured ? left.name() : "UNCONFIGURED");
      Logger.recordOutput(prefix + "/RIGHT", configured ? opposite(left).name() : "UNCONFIGURED");
    }
  }

  private Pattern selectedPattern() {
    // Accept stable enum keys as well as the human-readable labels used by the old chooser.
    String value = chooser.get();
    for (Pattern pattern : Pattern.values()) {
      if (pattern.name().equals(value) || pattern.label().equals(value)) {
        return pattern;
      }
    }
    return Pattern.UNCONFIGURED;
  }

  private static CoralState opposite(CoralState color) {
    return color == BLACK ? WHITE : BLACK;
  }

  // Names encode the LEFT colors in L4/L3/L2 order. Each RIGHT color is the opposite.
  private enum Pattern {
    UNCONFIGURED(null, null, null),
    BBB(BLACK, BLACK, BLACK),
    BBW(BLACK, BLACK, WHITE),
    BWB(BLACK, WHITE, BLACK),
    BWW(BLACK, WHITE, WHITE),
    WBB(WHITE, BLACK, BLACK),
    WBW(WHITE, BLACK, WHITE),
    WWB(WHITE, WHITE, BLACK),
    WWW(WHITE, WHITE, WHITE);

    private final CoralState leftL4;
    private final CoralState leftL3;
    private final CoralState leftL2;

    Pattern(CoralState leftL4, CoralState leftL3, CoralState leftL2) {
      this.leftL4 = leftL4;
      this.leftL3 = leftL3;
      this.leftL2 = leftL2;
    }

    CoralState leftColor(Level level) {
      return switch (level) {
        case L4 -> leftL4;
        case L3 -> leftL3;
        case L2 -> leftL2;
      };
    }

    String label() {
      if (this == UNCONFIGURED) return name();
      return "L4 " + leftL4 + "/" + opposite(leftL4)
          + " | L3 " + leftL3 + "/" + opposite(leftL3)
          + " | L2 " + leftL2 + "/" + opposite(leftL2);
    }
  }
}
