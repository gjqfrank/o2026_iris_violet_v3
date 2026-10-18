// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.spinindexer;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;

public final class SpinIndexerConstants {
  public static final boolean REAL_IO_CONFIGURED = false; // TODO: Enable after hardware, current limits and gains are verified.

  public static final Velocity<CurrentUnit> SYSID_RAMP_RATE = Amps.per(Second).of(1.0); // TODO: Validate the initial ramp with the arm clear of the turntable, A/s; 5 s reaches approximately 5 A.
  public static final Current SYSID_STEP_CURRENT = Amps.of(5.0); // TODO: Validate this initial torque-current step on the assembled turntable, A.
  public static final Time SYSID_QUASISTATIC_DURATION = Seconds.of(5.0); // TODO: Validate useful ramp data and safe turntable speed for empty and loaded tests, s.
  public static final Time SYSID_DYNAMIC_DURATION = Seconds.of(1.0); // TODO: Validate capture of the acceleration response and safe turntable speed, s.

  public static final int MOTOR_ID = 10;
  public static final int CANCODER_ID = 11;
  public static final String CAN_BUS_NAME = "-1"; // TODO: Set the CAN bus name.

  // Direction values use 0 for false and 1 for true. -1 means not configured.
  public static final int MOTOR_INVERTED = -1; // TODO: Confirm the positive motor direction.
  public static final int CANCODER_INVERTED = -1; // TODO: Confirm the positive sensor direction.
  public static final double CANCODER_OFFSET_ROTATIONS = -1.0; // TODO: Calibrate the zero offset.
  public static final double CANCODER_TO_MECHANISM_RATIO =
      1.0; // The WCP ThroughBore encoder directly measures the turntable shaft.

  public static final int SLOT_COUNT = 3;
  // Calibrated turntable angles at which each slot aligns with the arm's transfer position.
  // NaN deliberately disables slot positioning until all three angles have been measured.
  public static final Angle SLOT_1_ALIGNMENT_ANGLE = Degrees.of(Double.NaN); // TODO: Turntable shaft angle when slot 1 aligns with the arm transfer position; deg in the zeroed CANcoder reference with verified positive direction. Used for slot selection and inventory updates.
  public static final Angle SLOT_2_ALIGNMENT_ANGLE = Degrees.of(Double.NaN); // TODO: Turntable shaft angle when slot 2 aligns with the arm transfer position; use the same CANcoder zero and positive direction as slot 1, deg.
  public static final Angle SLOT_3_ALIGNMENT_ANGLE = Degrees.of(Double.NaN); // TODO: Turntable shaft angle when slot 3 aligns with the arm transfer position; use the same CANcoder zero and positive direction as slot 1, deg.
  public static final Angle SLOT_ALIGNMENT_TOLERANCE = Degrees.of(2.0); // TODO: Maximum absolute turntable angle error considered aligned for transfer and slot inventory updates, deg.
  public static final AngularVelocity SLOT_TRANSFER_MAX_VELOCITY =
      RotationsPerSecond.of(0.02); // TODO: Maximum absolute turntable shaft speed for transfer and slot inventory updates; checked together with angle tolerance, turntable rotations/s.


  public static final boolean BRAKE_MODE_ENABLED = true; // TODO: Verify disabled brake/coast behavior with the loaded turntable.
  public static final double PEAK_FORWARD_TORQUE_CURRENT_AMPS = 80.0;
  public static final double PEAK_REVERSE_TORQUE_CURRENT_AMPS = -80.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 80.0;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;

  // Torque-current closed-loop gains. TODO: Tune on the real mechanism.
  public static final double VELOCITY_KP = 0.0; // TODO: Tune velocity P using turntable rotations/s.
  public static final double VELOCITY_KI = 0.0; // TODO: Confirm whether velocity I is needed; 0 disables it.
  public static final double VELOCITY_KD = 0.0; // TODO: Tune velocity D or explicitly retain 0.
  public static final double VELOCITY_KS = 0.0; // TODO: Characterize static-friction current, A.
  public static final double VELOCITY_KV = 0.0; // TODO: Characterize velocity feedforward or explicitly retain 0.
  public static final double VELOCITY_KA = 0.0; // TODO: Characterize acceleration feedforward or explicitly retain 0.

  // Torque-current Motion Magic gains. TODO: Tune on the real mechanism.
  public static final double POSITION_KP = 0.0; // TODO: Tune Motion Magic position P using turntable rotations.
  public static final double POSITION_KI = 0.0; // TODO: Confirm whether position I is needed; 0 disables it.
  public static final double POSITION_KD = 0.0; // TODO: Tune position D or explicitly retain 0.
  public static final double POSITION_KS = 0.0; // TODO: Characterize static-friction current, A.
  public static final double POSITION_KV = 0.0; // TODO: Characterize profile velocity feedforward or explicitly retain 0.
  public static final double POSITION_KA = 0.0; // TODO: Characterize profile acceleration feedforward or explicitly retain 0.

  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0; // TODO: Validate turntable cruise velocity, rotations/s.
  public static final double MOTION_MAGIC_ACCELERATION = 8.0; // TODO: Validate turntable acceleration, rotations/s^2.
  public static final double MOTION_MAGIC_JERK = 80.0; // TODO: Validate turntable jerk, rotations/s^3.

  public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
  public static final double SIM_REDUCTION = 1.0; // TODO: Update when the reduction is known.
  public static final MomentOfInertia SIM_MOI = KilogramSquareMeters.of(0.001); // TODO: Estimate turntable output inertia including stored coral, kg*m^2.
  // TODO: Retune these simulation-only starting gains after configuring SIM_REDUCTION and SIM_MOI.
  public static final double SIM_VELOCITY_KP = 5.0; // TODO: Retune simulation velocity P after configuring model inertia/reduction.
  public static final double SIM_VELOCITY_KI = 0.0; // TODO: Validate simulation velocity I against measured response; 0 disables it.
  public static final double SIM_VELOCITY_KD = 0.0; // TODO: Validate simulation velocity damping; 0 disables it.
  public static final double SIM_VELOCITY_KS = 0.0; // TODO: Validate simulated static friction against the measured mechanism.
  public static final double SIM_VELOCITY_KV = 0.0; // TODO: Validate simulation velocity feedforward; 0 disables it.
  public static final double SIM_VELOCITY_KA = 0.0; // TODO: Validate simulation acceleration feedforward; 0 disables it.
  public static final double SIM_POSITION_KP = 40.0; // TODO: Retune simulation position P after configuring model inertia/reduction.
  public static final double SIM_POSITION_KI = 0.0; // TODO: Validate simulation position I against measured response; 0 disables it.
  public static final double SIM_POSITION_KD = 0.0; // TODO: Validate simulation position damping; 0 disables it.
  public static final double SIM_POSITION_KS = 0.0; // TODO: Validate simulated static friction against the measured mechanism.
  public static final double SIM_POSITION_KV = 0.0; // TODO: Validate simulation profile velocity feedforward; 0 disables it.
  public static final double SIM_POSITION_KA = 0.0; // TODO: Validate simulation profile acceleration feedforward; 0 disables it.

  private SpinIndexerConstants() {}

}
