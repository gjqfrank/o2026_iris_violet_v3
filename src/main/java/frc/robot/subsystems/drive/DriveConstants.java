// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.CurrentUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;

// TODO: PathPlanner does not support Commands v3 yet; all PathPlanner wiring is removed. The
// physical and Autopilot constants below are retained for when auto returns on a v3-compatible
// planner.

public final class DriveConstants {
  // TODO: Enable only after calibrating the new chassis and verifying CAN IDs/inversions.
  public static final boolean REAL_IO_CONFIGURED = false;

  public static final Velocity<CurrentUnit> SYSID_RAMP_RATE = Amps.per(Second).of(2.0); // TODO: Validate this initial quasistatic ramp on carpet, A/s per drive motor; 5 s reaches approximately 10 A.
  public static final Current SYSID_STEP_CURRENT = Amps.of(10.0); // TODO: Validate this initial dynamic step against traction and available travel distance, A per drive motor.
  public static final Time SYSID_QUASISTATIC_DURATION = Seconds.of(5.0); // TODO: Validate enough usable ramp data and floor space, s; hardware current limits remain active.
  public static final Time SYSID_DYNAMIC_DURATION = Seconds.of(1.5); // TODO: Validate capture of the acceleration response without exhausting floor space, s.
  public static final Time SYSID_SETTLE_TIME = Seconds.of(1.0); // TODO: Verify modules settle facing robot +X before current excitation, s.
  public static final AngularVelocity WHEEL_RADIUS_MAX_VELOCITY = RadiansPerSecond.of(0.25); // TODO: Validate wheel-radius test chassis angular speed, rad/s.
  public static final AngularAcceleration WHEEL_RADIUS_RAMP_RATE =
      RadiansPerSecondPerSecond.of(0.05); // TODO: Validate wheel-radius test angular acceleration if used, rad/s^2.

  // PathPlanner gains and path constraints were removed with the PathPlanner wiring (see header TODO).

  public static final double ROBOT_MASS_KG = -1; // TODO: Weigh the complete robot, kg.
  public static final double ROBOT_MOI_KG_METERS_SQ = -1; // TODO: Estimate yaw MOI from CAD, kg*m^2.
  public static final double WHEEL_COF = -1; // TODO: Determine wheel/carpet traction coefficient.

  // Tuning defaults
  public static final double DEADBAND = 0.1; // TODO: Validate joystick deadband against actual controller drift.
  public static final double ANGLE_KP = -1; // TODO: Retune chassis heading hold P.
  public static final double ANGLE_KD = 0.0; // TODO: Validate heading damping; retain 0 if unnecessary.
  // TODO: Determine heading profile limits on the assembled robot, rad/s and rad/s^2.
  public static final AngularVelocity ANGLE_MAX_VELOCITY = RadiansPerSecond.of(-1); // TODO: Validate heading profile maximum velocity, rad/s.
  public static final AngularAcceleration ANGLE_MAX_ACCELERATION =
      RadiansPerSecondPerSecond.of(-1); // TODO: Validate heading profile maximum acceleration, rad/s^2.
  // Per-motor limits in amps; both match modes use the same supply limit.
  public static final double SUPPLY_LIMIT_AUTO = 60.0;
  public static final double SUPPLY_LIMIT_TELEOP = 60.0;
  public static final double STATOR_LIMIT_DEFAULT = 60.0;

  // Framework-only Autopilot settings, retained for auto/config/AutopilotConfig.
  public static final LinearAcceleration AP_ACCELERATION = MetersPerSecondPerSecond.of(5.0); // TODO: Validate Autopilot profile acceleration on this chassis, m/s^2.
  public static final Velocity<LinearAccelerationUnit> AP_JERK =
      MetersPerSecondPerSecond.per(Second).of(2.0); // TODO: Validate Autopilot profile jerk, m/s^3.
  public static final Distance AP_ERROR_XY_METERS = Meters.of(0.02); // TODO: Validate Autopilot translation tolerance, m.
  public static final Angle AP_ERROR_THETA_DEGREES = Degrees.of(1.0); // TODO: Validate Autopilot heading tolerance, deg.
  public static final Distance AP_BEELINE_RADIUS_METERS = Meters.of(0.08); // TODO: Validate Autopilot straight-approach radius, m.

  private DriveConstants() {}
}
