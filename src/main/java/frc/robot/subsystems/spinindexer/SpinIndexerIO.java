// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.spinindexer;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SpinIndexerIO {
  @AutoLog
  public static class SpinIndexerIOInputs {
    public boolean hardwareConfigurationValid = false;
    public Angle position = Rotations.of(0.0);
    public Angle absolutePosition = Rotations.of(0.0);
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    /** Current telemetry availability only, independent of hardware configuration and motion. */
    public boolean supplyCurrentAvailable = false;
    /** Signed for energy analysis: positive motoring, negative regenerative braking. */
    public Current statorCurrent = Amps.of(0.0);
    public boolean statorCurrentAvailable = false;
    /** Raw motor rotor rad/s, independent of remote CANcoder feedback scaling. */
    public double rotorVelocityRadPerSec = Double.NaN;
    public Current torqueCurrent = Amps.of(0.0); // Signed measured current for SysId.
    public Temperature temperature = Celsius.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SpinIndexerIOInputs inputs) {}

  /** Runs the spin indexer using open-loop voltage control. */
  public default void setVoltage(Voltage voltage) {}

  /** Applies torque current directly for characterization, without position/velocity control. */
  public default void setTorqueCurrent(Current current) {}

  /** Runs the spin indexer using torque-current velocity control. */
  public default void setVelocity(AngularVelocity velocity) {}

  /** Runs the spin indexer to a position using torque-current Motion Magic control. */
  public default void setPosition(Angle position) {}

  /** Plays a music tone through the spin indexer motor. */
  public default boolean setMusicTone(Frequency frequency) {
    return false;
  }

  /** Stops music output from the spin indexer motor. */
  public default void stopMusicTone() {}

  /** Sets the torque-current velocity gains. */
  public default void setVelocityPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Sets the torque-current Motion Magic gains. */
  public default void setPositionPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Sets the Motion Magic profile constraints in mechanism rotations. */
  public default void setMotionMagicConfig(
      double cruiseVelocity, double acceleration, double jerk) {}

  public default void stop() {}
}
