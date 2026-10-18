// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.driverstation.RobotState;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.units.measure.Frequency;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.system.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.generated.TunerConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: PathPlanner does not support Commands v3 yet; all path following wiring (AutoBuilder,
// path controllers, feedforward overloads) is removed until a v3-compatible planner exists.

public class Drive implements Mechanism {
  private static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0; // TODO: Verify actual odometry frame rate and CAN load with all devices active, Hz.
  public static final double DRIVE_BASE_RADIUS = calculateDriveBaseRadius();

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules;
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("gyro disconnected","Disconnected gyro, using kinematics as fallback.", Level.HIGH);
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  // SIM world pose follows wheel motion; estimator resets/corrections never move the world.
  private Pose2d simulationPose = Pose2d.ZERO;
  // Retains the previous sample from each module for delta tracking.
  private final SwerveModulePosition[] lastModulePositions = createZeroModulePositions();
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO);
  private Frequency commandedMusicTone = Hertz.zero();
  private boolean musicToneActive = false;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    this.gyroIO = gyroIO;
    modules =
        new Module[] {
          new Module(flModuleIO, 0, TunerConstants.FrontLeft),
          new Module(frModuleIO, 1, TunerConstants.FrontRight),
          new Module(blModuleIO, 2, TunerConstants.BackLeft),
          new Module(brModuleIO, 3, TunerConstants.BackRight)
        };

    // Usage reporting for swerve template
    HAL.reportUsage("RobotDrive", "SwerveDrive_AdvantageKit");

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // WPILib's SysId API is voltage-typed: bridge its numeric waveform 1:1 to amps.
    // Fit measured DriveTorqueCurrentAmps against wheel rotations/s; resulting FF gains use amps.
    Logger.recordOutput("Drive/SysIdInputUnit", "A");
    sysId =
      new SysIdRoutine(
        new SysIdRoutine.Config(
                Volts.per(Second).of(DriveConstants.SYSID_RAMP_RATE.in(Amps.per(Second))),
                Volts.of(DriveConstants.SYSID_STEP_CURRENT.in(Amps)),
                DriveConstants.SYSID_QUASISTATIC_DURATION,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.SysIdMechanism(
          voltage -> runCurrentCharacterization(voltage.in(Volts)), null, this));
  }

  private void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    if (Constants.tuningMode && Constants.getMode() == Mode.REAL) {
      for (var module : modules) module.updateTuning();
    }

    // Stop moving when disabled
    if (RobotState.isDisabled() && !musicToneActive) {
      stopModules();
    }

    Logger.recordOutput("Drive/CommandedMusicTone", commandedMusicTone.in(Hertz), Hertz);
    Logger.recordOutput("Drive/MusicToneActive", musicToneActive);

    // Log empty setpoint states when disabled
    if (RobotState.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    updateOdometry();

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  private void updateOdometry() {
    // All signals are sampled together, so every module uses the same timestamps.
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    SwerveModulePosition[][] odometryPositions = new SwerveModulePosition[modules.length][];
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      odometryPositions[moduleIndex] = modules[moduleIndex].getOdometryPositions();
    }

    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
        modulePositions[moduleIndex] = odometryPositions[moduleIndex][i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                Meters.of(modulePositions[moduleIndex].distance
                    - lastModulePositions[moduleIndex].distance),
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (Constants.getMode() == Mode.SIM) {
        simulationPose = simulationPose.plus(kinematics.toTwist2d(moduleDeltas).exp());
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  public void resetModuleHeading(
      Rotation2d flHeading, Rotation2d frHeading, Rotation2d blHeading, Rotation2d brHeading) {
    kinematics.resetHeadings(flHeading, frHeading, blHeading, brHeading);
  }

  /** Runs at the requested robot-relative speeds without additional feedforward. */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities discreteSpeeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteSpeeds);
    setpointStates =
        SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", discreteSpeeds);

    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (var module : modules) {
      module.runCharacterization(output);
    }
  }

  /** Runs drive characterization using torque current output. */
  public void runCurrentCharacterization(double outputAmps) {
    Logger.recordOutput("Drive/SysIdCommandedCurrentAmps", outputAmps, Amps);
    for (var module : modules) {
      module.runCurrentCharacterization(outputAmps);
    }
  }

  /** Plays the same music tone through all eight drive and turn motors. */
  public boolean setMusicTone(Frequency frequency) {
    commandedMusicTone = frequency;
    musicToneActive = true;
    boolean succeeded = true;
    for (var module : modules) {
      succeeded &= module.setMusicTone(frequency);
    }
    Logger.recordOutput("Drive/MusicToneRequestSucceeded", succeeded);
    return succeeded;
  }

  /** Stops music output and neutralizes all drive and turn motors. */
  public void stopMusicTone() {
    for (var module : modules) {
      module.stopMusicTone();
    }
    commandedMusicTone = Hertz.zero();
    musicToneActive = false;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  // Neutralize outputs when disabled; stop() requests closed-loop zero speed.
  private void stopModules() {
    for (var module : modules) {
      module.stop();
    }
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      headings[i] = getModuleTranslations()[i].getAngle().get();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdSettle()
        .andThen(sysId.quasistatic(direction))
        .named("Drive:SysIdQuasistatic(" + direction + ")");
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdSettle()
        .andThen(sysId.dynamic(direction).withTimeout(DriveConstants.SYSID_DYNAMIC_DURATION))
        .named("Drive:SysIdDynamic(" + direction + ")");
  }

  /** Holds a zero-amp characterization output while the modules settle to the test heading. */
  private Command sysIdSettle() {
    return run(
            c -> {
              double start = Timer.getTimestamp();
              while (Timer.getTimestamp() - start
                  < DriveConstants.SYSID_SETTLE_TIME.in(Seconds)) {
                runCurrentCharacterization(0.0);
                c.yield();
              }
            })
        .named("Drive:SysIdSettle");
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured robot-relative chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (var module : modules) {
      output += module.getFFCharacterizationVelocity() / modules.length;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** True when the pose sits within the position and heading tolerances of the target. */
  public boolean isWithinPose(Pose2d target, double positionToleranceMeters, double angleToleranceRadians) {
    Pose2d pose = getPose();
    double distance = pose.getTranslation().getDistance(target.getTranslation());
    double angle = Math.abs(pose.getRotation().minus(target.getRotation()).getRadians());
    return distance <= positionToleranceMeters && angle <= angleToleranceRadians;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the estimated field-relative rotation at a timestamp, falling back to the latest. */
  public Rotation2d getRotationAtTimestamp(double timestampSeconds) {
    return poseEstimator
        .sampleAt(timestampSeconds)
        .map(Pose2d::getRotation)
        .orElseGet(this::getRotation);
  }

  /** Runs the drive modules to the specified heading to tune the turn PID gains. */
  public void runTurnPosition(Rotation2d heading) {
    for (var module : modules) {
      module.runTurnPosition(heading);
    }
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public Pose2d getSimulationPose() {
    return simulationPose;
  }

  /** Explicit SIM world reset, separate from resetting estimated localization. */
  public void resetSimulationPose(Pose2d pose) {
    if (Constants.getMode() != Mode.SIM) return;
    simulationPose = pose;
    setPose(pose);
  }

  /** Reset all steer motor positions to their absolute encoder positions. */
  public void resetAbsoluteTurnPosition() {
    for (var module : modules) {
      module.resetAbsoluteTurnPosition();
    }
  }

  /** Adds a new timestamped vision measurement with optional vision heading fusion. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      boolean updateHeading) {
    Pose2d measurementPose = visionRobotPoseMeters;
    if (!updateHeading) {
      measurementPose =
          new Pose2d(
              visionRobotPoseMeters.getTranslation(), getRotationAtTimestamp(timestampSeconds));
    }
    poseEstimator.addVisionMeasurement(measurementPose, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return MODULE_TRANSLATIONS.clone();
  }

  private static double calculateDriveBaseRadius() {
    double radius = 0.0;
    for (var translation : MODULE_TRANSLATIONS) {
      radius = Math.max(radius, translation.getNorm());
    }
    return radius;
  }

  private static SwerveModulePosition[] createZeroModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[MODULE_TRANSLATIONS.length];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = new SwerveModulePosition();
    }
    return positions;
  }

  public void updateSupplyCurrentLimit(double limit) {
    for (var module : modules) {
      module.updateSupplyCurrentLimit(limit);
    }
  }

  public void updateStatorCurrentLimit(double limit) {
    for (var module : modules) {
      module.updateStatorCurrentLimit(limit);
    }
  }

  /** Safety query for coordinated Reef actions; leaves normal driving behavior unchanged. */
  public boolean isMotionFeedbackHealthy(double maxAgeSeconds) {
    if (!Double.isFinite(maxAgeSeconds) || maxAgeSeconds <= 0
        || (Constants.getMode() != Mode.SIM && !gyroInputs.connected)) return false;
    if (Constants.getMode() != Mode.SIM
        && (!Double.isFinite(gyroInputs.motionFeedbackAgeSeconds)
            || gyroInputs.motionFeedbackAgeSeconds > maxAgeSeconds
            || !Double.isFinite(gyroInputs.yawVelocityRotPerSec))) return false;
    double now = Timer.getTimestamp();
    for (var module : modules) {
      if (!module.feedbackHealthy(now, maxAgeSeconds)) return false;
    }
    return true;
  }

  public double getMeasuredYawVelocityRadPerSec() {
    return Constants.getMode() == Mode.SIM ? getChassisVelocities().omega
        : gyroInputs.yawVelocityRotPerSec * 2.0 * Math.PI;
  }
}
