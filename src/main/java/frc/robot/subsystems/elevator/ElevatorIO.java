// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  enum Axis { HEIGHT, PIVOT }
  enum ControlState { STOPPED, POSITION, HOMING, MUSIC }

  /** Position gains use mechanism rotations and amps; homing gains are separately volts-based. */
  record ClosedLoopGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    public boolean isFinite() {
      return Double.isFinite(kP) && Double.isFinite(kI) && Double.isFinite(kD)
          && Double.isFinite(kS) && Double.isFinite(kV) && Double.isFinite(kA) && Double.isFinite(kG);
    }
  }

  @AutoLog
  class ElevatorIOInputs {
    public boolean hardwareConfigurationValid = false;
    public boolean heightReferenceValid = false;
    public boolean homingActive = false;
    public String mechanismFault = "";
    public Distance height = Meters.zero();
    public LinearVelocity heightVelocity = MetersPerSecond.zero();
    public Angle pivotAngle = Radians.zero();
    public AngularVelocity pivotVelocity = RadiansPerSecond.zero();
    public Voltage leaderAppliedVoltage = Volts.zero();
    public Current leaderSupplyCurrent = Amps.zero();
    /** Current telemetry availability only; never participates in motion or homing protection. */
    public boolean leaderSupplyCurrentAvailable = false;
    /** Signed for energy analysis: positive motoring, negative regenerative braking. */
    public Current leaderStatorCurrent = Amps.zero();
    public boolean leaderStatorCurrentAvailable = false;
    /** Raw motor rotor rad/s, independent of Average/Difference feedback scaling. */
    public double leaderRotorVelocityRadPerSec = Double.NaN;
    public Temperature leaderTemperature = Celsius.zero();
    public Voltage followerAppliedVoltage = Volts.zero();
    public Current followerSupplyCurrent = Amps.zero();
    public boolean followerSupplyCurrentAvailable = false;
    public Current followerStatorCurrent = Amps.zero();
    public boolean followerStatorCurrentAvailable = false;
    public double followerRotorVelocityRadPerSec = Double.NaN;
    public Temperature followerTemperature = Celsius.zero();
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setPosition(Distance height, Angle pivotAngle) {}
  default void setPIDGains(Axis axis, ClosedLoopGains gains) {}
  default void setMotionMagicConfig(double cruiseMetersPerSecond,
      double accelerationMetersPerSecondSquared, double jerkMetersPerSecondCubed) {}
  default void setPivotProfileConfig(double velocityRadiansPerSecond,
      double accelerationRadiansPerSecondSquared) {}

  /** Calibrates only height; the pivot's absolute reference must remain unchanged. */
  default boolean resetHeight(Distance height) { return false; }
  /** Starts a height homing session while holding the captured pivot angle. */
  default boolean beginHoming(Angle pivotAngle) { return false; }
  default boolean setHomingVoltage(Voltage voltage) { return false; }
  /** Only valid during an active session after the configured stall checks are satisfied. */
  default boolean completeHoming() { return false; }
  default boolean setMusicTone(Frequency frequency) { return false; }
  default void stopMusicTone() {}
  default void setBrakeModeEnabled(boolean enabled) {}
  default void stop() {}
}
