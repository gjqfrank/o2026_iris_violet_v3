// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot;

import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.Timer;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import frc.robot.energy.BatteryLogger;
import frc.robot.util.CANcoderBootValidator;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double SIM_AUTO_ENABLE_DELAY_SECS = 0.5;

  private Command autonomousCommand;
  private Command teleopCommand;
  private RobotContainer robotContainer;
  private final BatteryLogger batteryLogger;
  public static CANcoderBootValidator cancoderValidator;
  private final boolean simAutoEnable = "1".equals(System.getenv("SIM_AUTO_ENABLE"));
  private final BooleanEntry simRedAlliance =
      NetworkTableInstance.getDefault().getBooleanTopic("/Robot/Sim/RedAlliance").getEntry(false);
  private double simStartTimestamp = 0.0;
  // Tracks the last sim alliance override applied so we only push to the HAL when it changes.
  private Boolean simRedAllianceOverride = null;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Keep a local log for Cyber Power analysis as well as live SIM dashboard output.
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
    batteryLogger = BatteryLogger.getInstance();

    // CANcoder boot validation in real mode. Registration happens in RobotContainer.
    if (Constants.getMode() == Constants.Mode.REAL) {
      cancoderValidator = new CANcoderBootValidator();
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // In sim, force the HAL alliance from a SmartDashboard toggle. The FMS GUI widget does
    // not always update the HAL state reliably, so we override it here before any code reads
    // DriverStation.getAlliance() this cycle.
    if (Constants.getMode() == Constants.Mode.SIM) {
      /**
     * Reads the SmartDashboard "Sim/RedAlliance" toggle and pushes it to the HAL via
     * DriverStationSim. This runs every cycle in SIM so code that calls DriverStation.getAlliance()
     * picks up the override immediately, regardless of whether the sim GUI's FMS widget has published
     * fresh data.
     */
      boolean wantRed = simRedAlliance.get();
      if (simRedAllianceOverride == null || simRedAllianceOverride != wantRed) {
        DriverStationSim.setAllianceStationId(
            wantRed ? AllianceStationID.RED_1 : AllianceStationID.BLUE_1);
        DriverStationSim.notifyNewData();
        simRedAllianceOverride = wantRed;
      }
    }

    robotContainer.updateAlliance();
    batteryLogger.periodicBeforeScheduler(org.wpilib.system.RobotController.getBatteryVoltage());
    Scheduler.getDefault().run();
    batteryLogger.periodicAfterScheduler();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    cancelTeleopCommand();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    cancelTeleopCommand();
    robotContainer.applyAutoDriveCurrentLimit();
    robotContainer.initializeSpinIndexerPreload();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      Scheduler.getDefault().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    cancelTeleopCommand();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      Scheduler.getDefault().cancel(autonomousCommand);
    }
    robotContainer.applyTeleopDriveCurrentLimit();
    robotContainer.initializeCoralPreloadIfNeeded();
    teleopCommand = robotContainer.getTeleopCommand();
    Scheduler.getDefault().schedule(teleopCommand);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void utilityInit() {
    cancelTeleopCommand();
  }

  private void cancelTeleopCommand() {
    if (teleopCommand != null) {
      Scheduler.getDefault().cancel(teleopCommand);
      teleopCommand = null;
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    simStartTimestamp = Timer.getTimestamp();
    // Publish the sim alliance override entry with a default of false (blue) so the Elastic
    // Boolean Box widget has a topic to bind to before the first toggle.
    updateSimAutoEnable();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    updateSimAutoEnable();
  }

  private void updateSimAutoEnable() {
    if (!simAutoEnable) {
      return;
    }
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(
        Timer.getTimestamp() - simStartTimestamp >= SIM_AUTO_ENABLE_DELAY_SECS);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void close() {
    super.close();
  }
}
