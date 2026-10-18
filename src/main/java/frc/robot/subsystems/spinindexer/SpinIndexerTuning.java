// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.spinindexer;

import frc.robot.util.LoggedTunableNumber;

public final class SpinIndexerTuning {
  public static final LoggedTunableNumber VELOCITY_KP =
      new LoggedTunableNumber("SpinIndexer/VelocityKp", SpinIndexerConstants.VELOCITY_KP);
  public static final LoggedTunableNumber VELOCITY_KI =
      new LoggedTunableNumber("SpinIndexer/VelocityKi", SpinIndexerConstants.VELOCITY_KI);
  public static final LoggedTunableNumber VELOCITY_KD =
      new LoggedTunableNumber("SpinIndexer/VelocityKd", SpinIndexerConstants.VELOCITY_KD);
  public static final LoggedTunableNumber VELOCITY_KS =
      new LoggedTunableNumber("SpinIndexer/VelocityKs", SpinIndexerConstants.VELOCITY_KS);
  public static final LoggedTunableNumber VELOCITY_KV =
      new LoggedTunableNumber("SpinIndexer/VelocityKv", SpinIndexerConstants.VELOCITY_KV);
  public static final LoggedTunableNumber VELOCITY_KA =
      new LoggedTunableNumber("SpinIndexer/VelocityKa", SpinIndexerConstants.VELOCITY_KA);

  public static final LoggedTunableNumber POSITION_KP =
      new LoggedTunableNumber("SpinIndexer/PositionKp", SpinIndexerConstants.POSITION_KP);
  public static final LoggedTunableNumber POSITION_KI =
      new LoggedTunableNumber("SpinIndexer/PositionKi", SpinIndexerConstants.POSITION_KI);
  public static final LoggedTunableNumber POSITION_KD =
      new LoggedTunableNumber("SpinIndexer/PositionKd", SpinIndexerConstants.POSITION_KD);
  public static final LoggedTunableNumber POSITION_KS =
      new LoggedTunableNumber("SpinIndexer/PositionKs", SpinIndexerConstants.POSITION_KS);
  public static final LoggedTunableNumber POSITION_KV =
      new LoggedTunableNumber("SpinIndexer/PositionKv", SpinIndexerConstants.POSITION_KV);
  public static final LoggedTunableNumber POSITION_KA =
      new LoggedTunableNumber("SpinIndexer/PositionKa", SpinIndexerConstants.POSITION_KA);

  public static final LoggedTunableNumber MOTION_MAGIC_CRUISE_VELOCITY =
      new LoggedTunableNumber(
          "SpinIndexer/MotionMagicCruiseVelocity",
          SpinIndexerConstants.MOTION_MAGIC_CRUISE_VELOCITY);
  public static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "SpinIndexer/MotionMagicAcceleration",
          SpinIndexerConstants.MOTION_MAGIC_ACCELERATION);
  public static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "SpinIndexer/MotionMagicJerk", SpinIndexerConstants.MOTION_MAGIC_JERK);

  private SpinIndexerTuning() {}
}
