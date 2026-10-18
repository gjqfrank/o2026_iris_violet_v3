// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.util;

import java.util.Objects;

/** Builds consistently namespaced AdvantageKit log keys. */
public final class LogScope {
  private final String prefix;

  private LogScope(String prefix) {
    this.prefix = prefix;
  }

  /** Creates a scope below {@code Commands/<simple class name>}. */
  public static LogScope command(Class<?> commandClass) {
    Objects.requireNonNull(commandClass, "commandClass");
    return new LogScope("Commands/" + commandClass.getSimpleName());
  }

  /** Creates a scope from an explicitly named root. */
  public static LogScope of(String prefix) {
    return new LogScope(Objects.requireNonNull(prefix, "prefix"));
  }

  /** Returns a key below this scope. */
  public String key(String relativePath) {
    return join(prefix, relativePath);
  }

  /** Returns a key below this scope's debug namespace. */
  public String debugKey(String relativePath) {
    return join(key("Debug"), relativePath);
  }

  private static String join(String prefix, String relativePath) {
    Objects.requireNonNull(relativePath, "relativePath");
    return relativePath.isEmpty() ? prefix : prefix + "/" + relativePath;
  }
}
