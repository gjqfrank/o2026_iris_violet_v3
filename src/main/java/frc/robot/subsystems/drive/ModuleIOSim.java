// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.Timer;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared
  // locally
  private static final double DRIVE_KP = 0.05; // TODO: Retune this simulation-only velocity gain when model parameters are configured.
  private static final double DRIVE_KD = 0.0; // TODO: Validate simulation damping against measured response; 0 disables it.
  private static final double DRIVE_KS = 0.0; // TODO: Validate the frictionless simulation assumption; 0 disables static FF.
  private static final double TURN_KP = 8.0; // TODO: Retune this simulation-only turn gain when model parameters are configured.
  private static final double TURN_KD = 0.0; // TODO: Validate simulation turn damping against measured response; 0 disables it.
  // TODO: Verify the drive motor model matches the installed motor.
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX44Foc(1); // MK5n steering motor.

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  // Drive velocity feedforward derived from the sim plant itself (volts per wheel rad/s at free
  // speed), NOT the real-robot TunerConstants kV: a mismatched feedforward makes the sim modules
  // track ~20% above the commanded speed, which destabilizes any controller that feeds measured
  // chassis speeds back into its next command (e.g. Autopilot drive-to-pose).
  private final double driveKv;

  private boolean driveClosedLoop = false;
  private boolean driveCurrentControl = false;
  private double driveCurrentSetpointAmps = 0.0;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
            DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
            TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    // Exact for the frictionless DCMotorSim plant: 12 V yields free speed / gear ratio at the
    // wheel, so feedforward alone settles on the commanded velocity.
    driveKv = 12.0 / (DRIVE_GEARBOX.freeSpeed / constants.DriveMotorGearRatio);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
      if (driveCurrentControl) {
        double motorSpeed = driveSim.getAngularVelocity() * driveSim.getGearing();
        driveAppliedVolts = Math.clamp(
            DRIVE_GEARBOX.getVoltage(DRIVE_GEARBOX.getTorque(driveCurrentSetpointAmps), motorSpeed),
            -12.0, 12.0);
      }
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.motionFeedbackAvailable = true;
    inputs.motionFeedbackAgeSeconds = 0.0;
    inputs.drivePositionRot = org.wpilib.math.util.Units.radiansToRotations(driveSim.getAngularPosition());
    inputs.driveVelocityRotPerSec =
        org.wpilib.math.util.Units.radiansToRotations(driveSim.getAngularVelocity());
    inputs.driveAppliedVolts = driveAppliedVolts;
    double driveWindingCurrent = getStatorCurrentAmps(driveSim);
    inputs.driveEnergyTelemetryAvailable = true;
    inputs.driveRotorVelocityRadPerSec = driveSim.getAngularVelocity() * driveSim.getGearing();
    // Ideal controller DC current preserves regeneration and accounts for its voltage duty cycle.
    inputs.driveSupplyCurrentAmps = driveWindingCurrent * driveSim.getInputVoltage()
        / RobotController.getBatteryVoltage();
    inputs.driveStatorCurrentAmps = driveWindingCurrent * inputs.driveRotorVelocityRadPerSec < 0
        ? -Math.abs(driveWindingCurrent) : Math.abs(driveWindingCurrent);
    // Torque current remains direction-signed for SysId, independent of the energy convention.
    inputs.driveTorqueCurrentAmps = driveWindingCurrent;

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRotPerSec =
        org.wpilib.math.util.Units.radiansToRotations(turnSim.getAngularVelocity());
    inputs.turnAppliedVolts = turnAppliedVolts;
    double turnWindingCurrent = getStatorCurrentAmps(turnSim);
    inputs.turnEnergyTelemetryAvailable = true;
    inputs.turnRotorVelocityRadPerSec = turnSim.getAngularVelocity() * turnSim.getGearing();
    inputs.turnSupplyCurrentAmps = turnWindingCurrent * turnSim.getInputVoltage()
        / RobotController.getBatteryVoltage();
    inputs.turnStatorCurrentAmps = turnWindingCurrent * inputs.turnRotorVelocityRadPerSec < 0
        ? -Math.abs(turnWindingCurrent) : Math.abs(turnWindingCurrent);

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRot = new double[] {inputs.drivePositionRot};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  private static double getStatorCurrentAmps(DCMotorSim sim) {
    // DCMotorSim has no motor-controller/DC-link model. Reconstruct the signed winding current
    // from its motor state; the input voltage then supplies the ideal controller duty-cycle factor.
    return sim.getGearbox()
        .getCurrent(sim.getAngularVelocity() * sim.getGearing(), sim.getInputVoltage());
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveCurrentControl = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setDriveOpenLoopCurrent(double outputAmps) {
    driveClosedLoop = false;
    driveCurrentControl = true;
    driveCurrentSetpointAmps = outputAmps;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveCurrentControl = false;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforwardAmps) {
    setDriveVelocity(velocityRadPerSec); // no ff in sim
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
