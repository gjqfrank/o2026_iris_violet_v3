// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.constants;

import org.wpilib.framework.RobotBase;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Pattern;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Desktop runs use {@link #mode} for simulation or replay.
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final double kCANcoderBootTimeoutSeconds = 10.0; // TODO: Validate maximum CANcoder startup wait under full CAN load, s.
  public static final Mode mode = Mode.SIM;
  public static final boolean tuningMode = false;
  private static final Pattern TEAM_NUMBER_PATTERN =
      Pattern.compile("\"teamNumber\"\\s*:\\s*(\\d+)");

  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : mode;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Checks whether the correct team number is configured when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      checkTeamNumber("deploy");
    }
  }

  /** Checks team number and merge-sensitive toggles during local/CI verification. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (tuningMode) {
        System.err.println("Do not merge, tuning mode enabled.");
        System.exit(1);
      }
      checkTeamNumber("build");
    }
  }

  private static void checkTeamNumber(String phase) {
    int expectedTeamNumber = 6907;
    int actualTeamNumber = readTeamNumber(phase);
    if (actualTeamNumber != expectedTeamNumber) {
      System.err.println(
          "Cannot "
              + phase
              + ", teamNumber mismatch. Expected "
              + expectedTeamNumber
              + " but found "
              + actualTeamNumber
              + " in .wpilib/wpilib_preferences.json.");
      System.exit(1);
    }
  }

  private static int readTeamNumber(String phase) {
    Path preferencesPath = Path.of(".wpilib", "wpilib_preferences.json");
    try {
      String contents = Files.readString(preferencesPath, StandardCharsets.UTF_8);
      var matcher = TEAM_NUMBER_PATTERN.matcher(contents);
      if (matcher.find()) {
        return Integer.parseInt(matcher.group(1));
      }
    } catch (IOException e) {
      System.err.println(
          "Cannot "
              + phase
              + ", failed to read teamNumber from "
              + preferencesPath
              + ": "
              + e.getMessage());
      System.exit(1);
    }
    System.err.println("Cannot " + phase + ", teamNumber not found in " + preferencesPath + ".");
    System.exit(1);
    return -1;
  }
}
