// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.*;
import frc.robot.subsystems.elevator.ElevatorIO.ClosedLoopGains;

public final class ElevatorConstants {
  /** Height and pivot targets in the elevator's homed and calibrated pivot references. */
  public record ArmPose(Distance height, Angle pivot) {
    public boolean configured() {
      return Double.isFinite(height.in(Meters)) && Double.isFinite(pivot.in(Radians))
          && ElevatorConstants.mechanismConfigured()
          && height.in(Meters) >= ElevatorConstants.MIN_POSITION.in(Meters)
          && height.in(Meters) <= ElevatorConstants.MAX_POSITION.in(Meters)
          && pivot.in(Radians) >= ElevatorConstants.MIN_PIVOT_ANGLE.in(Radians)
          && pivot.in(Radians) <= ElevatorConstants.MAX_PIVOT_ANGLE.in(Radians);
    }
  }

  /** Enable only after resolving hardware, direction, calibration, limit, and gain TODOs. */
  public static final boolean REAL_IO_CONFIGURED = false; // TODO: Enable after differential hardware and control calibration.
  // Configure motor alignment and directions so positive Difference output increases pivot angle.
  public static final int LEADER_MOTOR_ID = 20;
  public static final int FOLLOWER_MOTOR_ID = 21;
  public static final int PIVOT_CANCODER_ID = 22;
  public static final String CAN_BUS_NAME = ""; // TODO: Shared CANivore name.

  // 0 = CCW positive, 1 = CW positive, -1 = unconfigured.
  public static final int LEADER_INVERTED = -1; // TODO: Positive Average output raises height.
  // 0 = Aligned, 1 = Opposed. Phoenix normalizes follower feedback/output using this alignment.
  // Physical shafts can turn oppositely while both normalized positions increase on Average.
  public static final int MOTOR_ALIGNMENT = -1; // TODO: Confirm actual belt routing.
  public static final int PIVOT_CANCODER_INVERTED = -1; // TODO: Positive Difference output increases angle.
  public static final Angle PIVOT_CANCODER_OFFSET =
      Rotations.of(Double.NaN); // TODO: Calibrate horizontal pivot to zero.
  public static final double PIVOT_CANCODER_DISCONTINUITY =
      Double.NaN; // TODO: In (0,1]; place the discontinuity outside the entire pivot travel.
  public static final double PIVOT_SENSOR_TO_MECHANISM_RATIO =
      1.0; // TODO: Confirm CANcoder directly measures the final pivot axis.
  public static final boolean BRAKE_MODE_ENABLED = true; // TODO: Verify disabled holding/coast behavior for both loaded axes.

  // Current limits apply to each of the two motors, in amps.
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
  public static final double PEAK_FORWARD_TORQUE_CURRENT_AMPS = 60.0;
  public static final double PEAK_REVERSE_TORQUE_CURRENT_AMPS = -60.0;
  /** Rotor turns per Average mechanism turn. */
  public static final double REDUCTION = 0.0; // TODO: Differential height reduction.
  /** Height change per Average mechanism turn, including elevator stage multipliers. */
  public static final Distance HEIGHT_PER_AVERAGE_ROTATION =
      Meters.of(Double.NaN); // TODO: Derive from belt routing, pulley diameter, and staging.
  public static final Distance HOME_POSITION = Meters.of(0.0); // Height origin at the designated manual-reset/homing position, m.
  public static final Distance MIN_POSITION = Meters.of(Double.NaN); // TODO: Conservative lower limit.
  public static final Distance MAX_POSITION = Meters.of(Double.NaN); // TODO: Conservative upper limit.
  public static final Distance POSITION_TOLERANCE = Meters.of(Double.NaN); // TODO: Validate height arrival tolerance, m.
  public static final Angle MIN_PIVOT_ANGLE = Degrees.of(Double.NaN); // TODO: Measure lower pivot limit with clearance, deg; horizontal is zero.
  public static final Angle MAX_PIVOT_ANGLE = Degrees.of(Double.NaN); // TODO: Measure upper pivot limit with clearance, deg; travel must be less than one turn.
  public static final Angle PIVOT_TOLERANCE = Degrees.of(Double.NaN); // TODO: Validate pivot arrival tolerance, deg.
  public static final LinearVelocity HEIGHT_ARRIVAL_VELOCITY_TOLERANCE =
      MetersPerSecond.of(0.02); // TODO: Validate settled height velocity tolerance for coral transfer, m/s.
  public static final AngularVelocity PIVOT_ARRIVAL_VELOCITY_TOLERANCE =
      RadiansPerSecond.of(Math.toRadians(2.0)); // TODO: Validate settled pivot velocity tolerance; input deg/s, stored rad/s.
  public static final Time ARRIVAL_SETTLE_TIME =
      Seconds.of(0.15); // TODO: Validate continuous arrival stability time before coral transfer, s.
  public static final LinearVelocity RESET_MAX_HEIGHT_VELOCITY =
      MetersPerSecond.of(Double.NaN); // TODO: Maximum motion allowed during manual reset.
  public static final AngularVelocity RESET_MAX_PIVOT_VELOCITY =
      RadiansPerSecond.of(Double.NaN); // TODO: Maximum motion allowed during manual reset.

  public static final double MOTION_MAGIC_CRUISE_VELOCITY_METERS_PER_SECOND = 0.0; // TODO: Validate safe height cruise velocity, m/s.
  public static final double MOTION_MAGIC_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.0; // TODO: Validate safe height acceleration, m/s^2.
  public static final double MOTION_MAGIC_JERK_METERS_PER_SECOND_CUBED = 0.0; // Intentionally disable jerk limiting.
  public static final double PIVOT_PROFILE_VELOCITY_RADIANS_PER_SECOND = 0.0; // TODO: Validate safe pivot profile velocity, rad/s.
  public static final double PIVOT_PROFILE_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 0.0; // TODO: Validate safe pivot profile acceleration, rad/s^2.
  public static final double CLOSED_LOOP_RATE_HZ = 100.0; // TODO: Verify differential feedback/control frame timing under full CAN load, Hz.

  // Native Slots 0/1 permanently hold height/pivot position gains in amps-based output units.
  // Height gains use Average rotations. Pivot gains use final pivot rotations.
  // kG: constant height gravity current; peak cosine gravity current for horizontal-zero pivot.
  public static final ClosedLoopGains HEIGHT_POSITION_GAINS =
      new ClosedLoopGains(0, 0, 0, 0, 0, 0, 0); // TODO: Tune.
  public static final ClosedLoopGains PIVOT_POSITION_GAINS =
      new ClosedLoopGains(0, 0, 0, 0, 0, 0, 0); // TODO: Tune.
  public static final ClosedLoopGains SIM_HEIGHT_POSITION_GAINS =
      new ClosedLoopGains(0, 0, 0, 0, 0, 0, 0); // TODO: Tune simulation.
  public static final ClosedLoopGains SIM_PIVOT_POSITION_GAINS =
      new ClosedLoopGains(0, 0, 0, 0, 0, 0, 0); // TODO: Tune simulation.

  public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(2);
  public static final Mass SIM_CARRIAGE_MASS = Kilograms.of(Double.NaN); // TODO: Determine equivalent lifted mass for the height model, kg.
  public static final double SIM_PIVOT_REDUCTION = Double.NaN; // TODO: Rotor/difference pivot turns.
  public static final double SIM_PIVOT_MOI_KG_METERS_SQUARED = Double.NaN; // TODO: Estimate pivot output-axis inertia, kg*m^2.
  public static final Distance SIM_PIVOT_LENGTH = Meters.of(Double.NaN); // TODO: Determine equivalent arm length for the pivot gravity model, m.
  public static final Angle SIM_INITIAL_PIVOT_ANGLE = Degrees.of(Double.NaN); // TODO: Choose initial pivot angle within configured limits, deg.
  public static final boolean SIMULATE_GRAVITY = true; // TODO: Validate the height/pivot gravity model against loaded mechanism response.

  /** Debug homing is independent of normal position control and manual toggle reset. */
  public static final boolean HOMING_CONFIGURED = false; // TODO: Validate on the actual mechanism.
  public static final Voltage HOMING_VOLTAGE = Volts.of(0.0); // TODO: Safe negative height voltage.
  public static final Current HOMING_CURRENT_THRESHOLD = Amps.of(0.0); // TODO: Both motors.
  public static final LinearVelocity HOMING_VELOCITY_THRESHOLD =
      MetersPerSecond.of(0.0); // TODO: Maximum height velocity considered stalled.
  public static final Time HOMING_START_DELAY = Seconds.of(0.0); // TODO: Ignore initial transients.
  public static final Time HOMING_STALL_TIME = Seconds.of(0.0); // TODO: Continuous stall duration.
  public static final Time HOMING_TIMEOUT = Seconds.of(0.0); // TODO: Maximum drive duration.
  public static final Angle HOMING_MAX_PIVOT_ERROR =
      Degrees.of(Double.NaN); // TODO: Abort if the captured pivot angle is not maintained.
  /** Volts-based gains, separate from the normal amps-based pivot gains; held angle has zero velocity. */
  public static final ClosedLoopGains HOMING_PIVOT_VOLTAGE_GAINS =
      new ClosedLoopGains(0, 0, 0, 0, 0, 0, 0); // TODO: Tune position PID and cosine kG in volts.

  private ElevatorConstants() {}

  public static boolean mechanismConfigured() {
    double min = MIN_POSITION.in(Meters), max = MAX_POSITION.in(Meters);
    double minAngle = MIN_PIVOT_ANGLE.in(Rotations), maxAngle = MAX_PIVOT_ANGLE.in(Rotations);
    return positive(REDUCTION) && positive(HEIGHT_PER_AVERAGE_ROTATION.in(Meters))
        && Double.isFinite(min) && Double.isFinite(max) && max > min
        && within(HOME_POSITION.in(Meters), min, max)
        && positive(POSITION_TOLERANCE.in(Meters)) && positive(PIVOT_TOLERANCE.in(Radians))
        && positive(HEIGHT_ARRIVAL_VELOCITY_TOLERANCE.in(MetersPerSecond))
        && positive(PIVOT_ARRIVAL_VELOCITY_TOLERANCE.in(RadiansPerSecond))
        && positive(ARRIVAL_SETTLE_TIME.in(Seconds))
        && Double.isFinite(minAngle) && Double.isFinite(maxAngle)
        && maxAngle > minAngle && maxAngle - minAngle < 1.0
        && positive(MOTION_MAGIC_CRUISE_VELOCITY_METERS_PER_SECOND)
        && positive(MOTION_MAGIC_ACCELERATION_METERS_PER_SECOND_SQUARED)
        && Double.isFinite(MOTION_MAGIC_JERK_METERS_PER_SECOND_CUBED)
        && MOTION_MAGIC_JERK_METERS_PER_SECOND_CUBED >= 0
        && positive(PIVOT_PROFILE_VELOCITY_RADIANS_PER_SECOND)
        && positive(PIVOT_PROFILE_ACCELERATION_RADIANS_PER_SECOND_SQUARED)
        && positive(RESET_MAX_HEIGHT_VELOCITY.in(MetersPerSecond))
        && positive(RESET_MAX_PIVOT_VELOCITY.in(RadiansPerSecond));
  }

  public static double metersPerAverageRotation() {
    return HEIGHT_PER_AVERAGE_ROTATION.in(Meters);
  }
  public static boolean homingConfigured() {
    return HOMING_CONFIGURED && mechanismConfigured()
        && Double.isFinite(HOMING_VOLTAGE.in(Volts))
        && HOMING_VOLTAGE.in(Volts) < 0 && HOMING_VOLTAGE.in(Volts) >= -12
        && positive(HOMING_CURRENT_THRESHOLD.in(Amps))
        && Double.isFinite(HOMING_VELOCITY_THRESHOLD.in(MetersPerSecond))
        && HOMING_VELOCITY_THRESHOLD.in(MetersPerSecond) >= 0
        && HOMING_VELOCITY_THRESHOLD.in(MetersPerSecond)
            <= RESET_MAX_HEIGHT_VELOCITY.in(MetersPerSecond)
        && positive(HOMING_START_DELAY.in(Seconds)) && positive(HOMING_STALL_TIME.in(Seconds))
        && positive(HOMING_TIMEOUT.in(Seconds))
        && HOMING_TIMEOUT.in(Seconds)
            > HOMING_START_DELAY.in(Seconds) + HOMING_STALL_TIME.in(Seconds)
        && positive(HOMING_MAX_PIVOT_ERROR.in(Radians))
        && HOMING_PIVOT_VOLTAGE_GAINS.isFinite() && HOMING_PIVOT_VOLTAGE_GAINS.kP() > 0;
  }
  private static boolean positive(double x) { return Double.isFinite(x) && x > 0.0; }
  private static boolean within(double x, double min, double max) {
    return Double.isFinite(x) && x >= min && x <= max;
  }
}
