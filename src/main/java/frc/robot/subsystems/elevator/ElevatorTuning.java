// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunableNumber;

public final class ElevatorTuning {
  public static final LoggedTunableNumber POSITION_KP =
      new LoggedTunableNumber("Elevator/PositionKp",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kP());
  public static final LoggedTunableNumber POSITION_KI =
      new LoggedTunableNumber("Elevator/PositionKi",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kI());
  public static final LoggedTunableNumber POSITION_KD =
      new LoggedTunableNumber("Elevator/PositionKd",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kD());
  public static final LoggedTunableNumber POSITION_KS =
      new LoggedTunableNumber("Elevator/PositionKs",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kS());
  public static final LoggedTunableNumber POSITION_KV =
      new LoggedTunableNumber("Elevator/PositionKv",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kV());
  public static final LoggedTunableNumber POSITION_KA =
      new LoggedTunableNumber("Elevator/PositionKa",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kA());
  public static final LoggedTunableNumber POSITION_KG =
      new LoggedTunableNumber("Elevator/PositionKg",
          ElevatorConstants.HEIGHT_POSITION_GAINS.kG());

  public static final LoggedTunableNumber SIM_POSITION_KP =
      new LoggedTunableNumber("Elevator/SimPositionKp",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kP());
  public static final LoggedTunableNumber SIM_POSITION_KI =
      new LoggedTunableNumber("Elevator/SimPositionKi",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kI());
  public static final LoggedTunableNumber SIM_POSITION_KD =
      new LoggedTunableNumber("Elevator/SimPositionKd",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kD());
  public static final LoggedTunableNumber SIM_POSITION_KS =
      new LoggedTunableNumber("Elevator/SimPositionKs",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kS());
  public static final LoggedTunableNumber SIM_POSITION_KV =
      new LoggedTunableNumber("Elevator/SimPositionKv",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kV());
  public static final LoggedTunableNumber SIM_POSITION_KA =
      new LoggedTunableNumber("Elevator/SimPositionKa",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kA());
  public static final LoggedTunableNumber SIM_POSITION_KG =
      new LoggedTunableNumber("Elevator/SimPositionKg",
          ElevatorConstants.SIM_HEIGHT_POSITION_GAINS.kG());

  public static final LoggedTunableNumber PIVOT_POSITION_KP =
      new LoggedTunableNumber("Elevator/PivotPositionKp",
          ElevatorConstants.PIVOT_POSITION_GAINS.kP());
  public static final LoggedTunableNumber PIVOT_POSITION_KI =
      new LoggedTunableNumber("Elevator/PivotPositionKi",
          ElevatorConstants.PIVOT_POSITION_GAINS.kI());
  public static final LoggedTunableNumber PIVOT_POSITION_KD =
      new LoggedTunableNumber("Elevator/PivotPositionKd",
          ElevatorConstants.PIVOT_POSITION_GAINS.kD());
  public static final LoggedTunableNumber PIVOT_POSITION_KS =
      new LoggedTunableNumber("Elevator/PivotPositionKs",
          ElevatorConstants.PIVOT_POSITION_GAINS.kS());
  public static final LoggedTunableNumber PIVOT_POSITION_KV =
      new LoggedTunableNumber("Elevator/PivotPositionKv",
          ElevatorConstants.PIVOT_POSITION_GAINS.kV());
  public static final LoggedTunableNumber PIVOT_POSITION_KA =
      new LoggedTunableNumber("Elevator/PivotPositionKa",
          ElevatorConstants.PIVOT_POSITION_GAINS.kA());
  public static final LoggedTunableNumber PIVOT_POSITION_KG =
      new LoggedTunableNumber("Elevator/PivotPositionKg",
          ElevatorConstants.PIVOT_POSITION_GAINS.kG());

  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KP =
      new LoggedTunableNumber("Elevator/SimPivotPositionKp",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kP());
  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KI =
      new LoggedTunableNumber("Elevator/SimPivotPositionKi",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kI());
  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KD =
      new LoggedTunableNumber("Elevator/SimPivotPositionKd",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kD());
  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KS =
      new LoggedTunableNumber("Elevator/SimPivotPositionKs",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kS());
  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KV =
      new LoggedTunableNumber("Elevator/SimPivotPositionKv",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kV());
  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KA =
      new LoggedTunableNumber("Elevator/SimPivotPositionKa",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kA());
  public static final LoggedTunableNumber SIM_PIVOT_POSITION_KG =
      new LoggedTunableNumber("Elevator/SimPivotPositionKg",
          ElevatorConstants.SIM_PIVOT_POSITION_GAINS.kG());

  public static final LoggedTunableNumber MOTION_MAGIC_CRUISE_VELOCITY =
      new LoggedTunableNumber(
          "Elevator/MotionMagicCruiseVelocityMetersPerSecond",
          ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY_METERS_PER_SECOND);
  public static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Elevator/MotionMagicAccelerationMetersPerSecondSquared",
          ElevatorConstants.MOTION_MAGIC_ACCELERATION_METERS_PER_SECOND_SQUARED);
  public static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Elevator/MotionMagicJerkMetersPerSecondCubed",
          ElevatorConstants.MOTION_MAGIC_JERK_METERS_PER_SECOND_CUBED);
  public static final LoggedTunableNumber PIVOT_VELOCITY_LIMIT =
      new LoggedTunableNumber(
          "Elevator/Pivot/ProfileRadiansPerSecond",
          ElevatorConstants.PIVOT_PROFILE_VELOCITY_RADIANS_PER_SECOND);
  public static final LoggedTunableNumber PIVOT_ACCELERATION =
      new LoggedTunableNumber(
          "Elevator/Pivot/ProfileRadiansPerSecondSquared",
          ElevatorConstants.PIVOT_PROFILE_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

  private ElevatorTuning() {}
}
