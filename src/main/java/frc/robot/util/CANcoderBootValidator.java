// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.util;

import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import org.wpilib.units.measure.Angle;
import org.wpilib.system.Timer;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Waits for registered CANcoders to publish fresh absolute-position frames before drivetrain
 * construction. This reduces startup races where steer motors fuse against stale encoder data.
 */
public class CANcoderBootValidator {
  private static final int REQUIRED_VALID_UPDATES = 10; // TODO: Validate the startup fresh-frame count with installed CANcoders.

  private static class TimestampObserver {
    private final StatusSignal<Angle> positionSignal;
    private double lastTimestamp = -1.0;
    private int validUpdateCount = 0;

    TimestampObserver(CANcoder cancoder) {
      positionSignal = cancoder.getAbsolutePosition();
    }

    boolean hasValidUpdates() {
      positionSignal.refresh();
      double currentTimestamp = positionSignal.getTimestamp().getTime();

      if (currentTimestamp > lastTimestamp && lastTimestamp >= 0.0) {
        validUpdateCount++;
      }
      lastTimestamp = currentTimestamp;
      return validUpdateCount >= REQUIRED_VALID_UPDATES;
    }

    int getValidUpdateCount() {
      return validUpdateCount;
    }
  }

  private final Map<String, TimestampObserver> observers = new HashMap<>();

  public void registerCANcoder(String name, CANcoder cancoder) {
    observers.put(name, new TimestampObserver(cancoder));
  }

  /**
   * @return true when all registered CANcoders publish enough fresh updates before timeout.
   */
  public boolean waitForBoot(double timeoutSeconds) {
    double startTime = Timer.getTimestamp();
    Logger.recordOutput("CANcoderBoot/StartTime", startTime, Seconds);

    while (Timer.getTimestamp() - startTime < timeoutSeconds) {
      boolean allBooted = true;
      for (var entry : observers.entrySet()) {
        boolean hasBooted = entry.getValue().hasValidUpdates();
        Logger.recordOutput(
            "CANcoderBoot/" + entry.getKey() + "/ValidUpdates",
            entry.getValue().getValidUpdateCount());
        Logger.recordOutput("CANcoderBoot/" + entry.getKey() + "/HasBooted", hasBooted);
        if (!hasBooted) {
          allBooted = false;
        }
      }

      if (allBooted) {
        double bootTime = Timer.getTimestamp() - startTime;
        Logger.recordOutput("CANcoderBoot/BootTime", bootTime, Seconds);
        Logger.recordOutput("CANcoderBoot/Success", true);
        System.out.println("All CANcoders booted successfully in " + bootTime + " seconds");
        return true;
      }

      Timer.delay(0.02);
    }

    Logger.recordOutput("CANcoderBoot/BootTime", timeoutSeconds, Seconds);
    Logger.recordOutput("CANcoderBoot/Success", false);
    System.out.println("WARNING: CANcoder boot timeout after " + timeoutSeconds + " seconds");
    for (Map.Entry<String, TimestampObserver> entry : observers.entrySet()) {
      if (!entry.getValue().hasValidUpdates()) {
        System.out.println(
            "  - "
                + entry.getKey()
                + " failed to boot (only "
                + entry.getValue().getValidUpdateCount()
                + " valid updates)");
      }
    }
    return false;
  }
}
