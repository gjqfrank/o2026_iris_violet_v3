// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    StatusCode status = StatusCode.ConfigFailed;
    for (int i = 0; i < maxAttempts; i++) {
      status = command.get();
      if (status.isOK()) break;
    }
    return status;
  }
}
