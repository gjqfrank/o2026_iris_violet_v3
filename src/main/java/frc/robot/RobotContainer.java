// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot;

import java.util.List;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.RobotState;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringArrayPublisher;
import org.wpilib.networktables.StringEntry;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.sysid.SysIdRoutine;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.HomeResetCommand;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.ReefAlignConstants;
import frc.robot.constants.ReefAlignConstants.ReefBranch;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.CoralSceneSim;
import frc.robot.sim.MechanismVisualization;
import frc.robot.sim.SimConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.coralsensor.CANrangeIO;
import frc.robot.subsystems.coralsensor.CANrangeIOPhoenix;
import frc.robot.subsystems.coralsensor.CANrangeIOSim;
import frc.robot.subsystems.coralsensor.ColorSensorIO;
import frc.robot.subsystems.coralsensor.ColorSensorIOREV;
import frc.robot.subsystems.coralsensor.ColorSensorIOSim;
import frc.robot.subsystems.coralsensor.CoralSensor;
import frc.robot.subsystems.coralsensor.CoralSensorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOHardware;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.spinindexer.CoralState;
import frc.robot.subsystems.spinindexer.SpinIndexer;
import frc.robot.subsystems.spinindexer.SpinIndexerConstants;
import frc.robot.subsystems.spinindexer.SpinIndexerIO;
import frc.robot.subsystems.spinindexer.SpinIndexerIOSim;
import frc.robot.subsystems.spinindexer.SpinIndexerIOTalonFX;
import frc.robot.subsystems.toggles.ToggleConstants;
import frc.robot.subsystems.toggles.Toggles;
import frc.robot.subsystems.toggles.TogglesIO;
import frc.robot.subsystems.toggles.TogglesIOHardware;
import frc.robot.subsystems.toggles.TogglesIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ReefTargetColors;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive drive;
  private final SpinIndexer spinIndexer;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Toggles toggles;
  private final Vision vision;
  private final VisionSystemSim simulationVision;
  private final CoralSceneSim simulationCorals;
  private boolean redAlliance = AllianceFlipUtil.shouldFlip();
  private final ReefTargetColors reefTargetColors = new ReefTargetColors();
  private final List<EnumSelector<CoralState>> spinIndexerPreloadChoosers =
      List.of(
          new EnumSelector<>("/RobotContainer/Preload/SpinIndexer/Slot1", CoralState.EMPTY, CoralState.values()),
          new EnumSelector<>("/RobotContainer/Preload/SpinIndexer/Slot2", CoralState.EMPTY, CoralState.values()),
          new EnumSelector<>("/RobotContainer/Preload/SpinIndexer/Slot3", CoralState.EMPTY, CoralState.values()));
  private boolean spinIndexerPreloadInitialized = false;
  private final EnumSelector<CoralState> gripperPreloadChooser =
      new EnumSelector<>("/RobotContainer/Preload/EndEffector", CoralState.EMPTY, CoralState.values());
  private final CoralSensor coralSensor;
  private final Superstructure superstructure;
  private final StringEntry autoSelection =
      NetworkTableInstance.getDefault().getStringTopic("/RobotContainer/Auto/Selected").getEntry("Do Nothing");
  private final BooleanEntry resetSceneRequest =
      NetworkTableInstance.getDefault().getBooleanTopic("/RobotContainer/Sim/ResetScene").getEntry(false);
  private final BooleanEntry confirmInventoryRequest =
      NetworkTableInstance.getDefault().getBooleanTopic("/RobotContainer/Inventory/Confirm").getEntry(false);
  private final Map<String, Supplier<Command>> autoOptions = new LinkedHashMap<>();
    private final StringArrayPublisher autoOptionsPublisher =
      NetworkTableInstance.getDefault().getStringArrayTopic("/RobotContainer/Auto/Options").publish();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    EndEffectorIOSim simulatedJaw = null;
    ColorSensorIOSim simulatedGripperColor = null;
    ColorSensorIOSim simulatedIndexerColor = null;
    CANrangeIOSim simulatedGripperRange = null;
    CANrangeIOSim simulatedIndexerRange = null;
    switch (Constants.getMode()) {
      case REAL:
        // Wait for swerve CANcoders to report fresh absolute-position frames before
        // constructing modules that use fused feedback.
        if (DriveConstants.REAL_IO_CONFIGURED && Robot.cancoderValidator != null) {
          Robot.cancoderValidator.registerCANcoder(
              "FrontLeft",
              new CANcoder(TunerConstants.FrontLeft.EncoderId, TunerConstants.kCANBus));
          Robot.cancoderValidator.registerCANcoder(
              "FrontRight",
              new CANcoder(TunerConstants.FrontRight.EncoderId, TunerConstants.kCANBus));
          Robot.cancoderValidator.registerCANcoder(
              "BackLeft", new CANcoder(TunerConstants.BackLeft.EncoderId, TunerConstants.kCANBus));
          Robot.cancoderValidator.registerCANcoder(
              "BackRight",
              new CANcoder(TunerConstants.BackRight.EncoderId, TunerConstants.kCANBus));
          Robot.cancoderValidator.waitForBoot(Constants.kCANcoderBootTimeoutSeconds);
        }

        drive = createRealDrive();
        toggles = createRealToggles();
        elevator = createRealElevator();
        coralSensor = createRealCoralSensor();
        spinIndexer = createRealSpinIndexer(elevator::getHeight);
        endEffector = createRealEndEffector();
        break;
      
      case SIM:
        drive = new Drive(
        new GyroIO() {},
        createSimModule(TunerConstants.FrontLeft),
        createSimModule(TunerConstants.FrontRight),
        createSimModule(TunerConstants.BackLeft),
        createSimModule(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSim());
        spinIndexer = new SpinIndexer(new SpinIndexerIOSim() {}, elevator::getHeight);
        toggles = new Toggles(new TogglesIOSim() {});
        simulatedJaw = new EndEffectorIOSim();
        endEffector = new EndEffector(simulatedJaw);
        simulatedGripperColor = new ColorSensorIOSim();
        simulatedIndexerColor = new ColorSensorIOSim(SimConstants.INDEXER_SCAN_WHITE_CHANNEL);
        simulatedGripperRange = new CANrangeIOSim();
        simulatedIndexerRange = new CANrangeIOSim();
        coralSensor = new CoralSensor(CoralSensorConstants.GRIPPER, CoralSensorConstants.INDEXER,
            simulatedGripperRange, simulatedGripperColor, simulatedIndexerRange, simulatedIndexerColor);
        break;
      
      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        drive = new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        spinIndexer = new SpinIndexer(new SpinIndexerIO() {}, elevator::getHeight);
        toggles = new Toggles(new TogglesIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        coralSensor = new CoralSensor(CoralSensorConstants.GRIPPER, CoralSensorConstants.INDEXER,
            new CANrangeIO() {}, new ColorSensorIO() {}, new CANrangeIO() {}, new ColorSensorIO() {});
    }
    if (Constants.getMode() == Constants.Mode.SIM) {
      simulationVision = new VisionSystemSim("Reef");
      simulationVision.addAprilTags(SimConstants.REEF_LAYOUT);
      simulationCorals = new CoralSceneSim(drive, elevator, spinIndexer,
          simulatedJaw, simulatedGripperRange, simulatedGripperColor,
          simulatedIndexerRange, simulatedIndexerColor, reefTargetColors);
      resetSimulationPoseToOrigin();
      new Trigger(resetSceneRequest::get).onTrue(instant("Sim:ResetScene", () -> {
        resetSceneRequest.set(false);
        if (RobotState.isDisabled()) {
          resetSimulationPoseToOrigin();
          simulationCorals.resetReef();
          applyPreloadSelections();
        }
      }));
    } else {
      simulationVision = null;
      simulationCorals = null;
    }
    var cameraConfigs = simulationVision == null
        ? VisionConstants.cameraConfigs : SimConstants.CAMERA_CONFIGS;
    vision = new Vision(drive::addVisionMeasurement, cameraConfigs, createVisionIO(cameraConfigs));
    superstructure = new Superstructure(
        drive, elevator, spinIndexer, endEffector, coralSensor);
    Scheduler.getDefault().addPeriodic(this::periodic);
    configureAutoChooser();
    new Trigger(confirmInventoryRequest::get).onTrue(instant("Superstructure:ConfirmInventory", () -> {
      confirmInventoryRequest.set(false);
      // No requirements: an invalid enabled confirmation must not cancel a live transfer.
      if (RobotState.isDisabled() && !superstructure.isBusy()) applyPreloadSelections();
    }));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
  * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
  * predicate, or the named factories on {@link CommandXboxController}.
   */
  private void configureBindings() {
    var driverController = superstructure.gamepad();
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
        drive,
        () -> driveInputLocked() ? 0.0 : -driverController.getLeftY(),
        () -> driveInputLocked() ? 0.0 : -driverController.getLeftX(),
        () -> driveInputLocked() ? 0.0 : -driverController.getRightX(),
        () -> drive.getMaxLinearSpeedMetersPerSec()));
    
    // ReefAlign is a separate chassis action; the same Superstructure-owned gamepad supplies its bumpers.
    Trigger leftAlign = driverController.leftBumper();
    Trigger rightAlign = driverController.rightBumper();
    Trigger driveAvailable = new Trigger(() -> !superstructure.isDriveLocked());
    leftAlign.and(rightAlign.negate()).and(driveAvailable).whileTrue(new ReefAlignCommand(
        drive, ReefBranch.LEFT,
        () -> vision.hasRecentPoseMeasurement(ReefAlignConstants.maxVisionAgeSeconds),
        superstructure::observeAlignment));
    rightAlign.and(leftAlign.negate()).and(driveAvailable).whileTrue(new ReefAlignCommand(
        drive, ReefBranch.RIGHT,
        () -> vision.hasRecentPoseMeasurement(ReefAlignConstants.maxVisionAgeSeconds),
        superstructure::observeAlignment));
    leftAlign.and(rightAlign).and(driveAvailable).whileTrue(
        drive.run(
                coroutine -> {
                  superstructure.observeAlignment(null);
                  while (true) {
                    drive.stop();
                    coroutine.yield();
                  }
                })
            .named("Drive:StopForBothReefBumpers"));

      Toggles.createDebouncedTriggerOnChange(
        toggles::isBrakeModeButtonPressed,
        brakeModeEnabled -> {
          elevator.setBrakeModeEnabled(brakeModeEnabled);
          System.out.println(
              "Mechanism brake mode button changed (debounced) - set to "
                  + (brakeModeEnabled ? "brake" : "coast")
                  + " mode");
        });

    Toggles.createDebouncedTriggerOnTrue(
        toggles::isHomeButtonPressed,
        HomeResetCommand.create(drive, spinIndexer, elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Supplier<Command> selected = autoOptions.get(autoSelection.get());
    Command selectedCommand = selected == null ? instant("Auto:None", () -> {}) : selected.get();
    return Command.noRequirements(
        coroutine -> {
          coroutine.await(selectedCommand);
          drive.stop();
        })
      .whenCanceled(drive::stop)
      .whenExited(drive::stop)
      .named("Autonomous");
  }

  private void configureAutoChooser() {
    autoOptions.put("Do Nothing", () -> instant("Auto:None", () -> {}));
    autoOptions.put("Drive Wheel Radius Characterization",
      () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoOptions.put("Drive Current SysId - Quasistatic Forward",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
    autoOptions.put("Drive Current SysId - Quasistatic Reverse",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
    autoOptions.put("Drive Current SysId - Dynamic Forward",
        () -> drive.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
    autoOptions.put("Drive Current SysId - Dynamic Reverse",
        () -> drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
    autoOptions.put("SpinIndexer Current SysId - Quasistatic Forward",
        () -> spinIndexer.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
    autoOptions.put("SpinIndexer Current SysId - Quasistatic Reverse",
        () -> spinIndexer.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
    autoOptions.put("SpinIndexer Current SysId - Dynamic Forward",
        () -> spinIndexer.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
    autoOptions.put("SpinIndexer Current SysId - Dynamic Reverse",
        () -> spinIndexer.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
    autoOptionsPublisher.set(autoOptions.keySet().toArray(String[]::new));
  }

  /** Restore the auto power budget even when practice starts auto after a Teleop period. */
  public void applyAutoDriveCurrentLimit() {
    drive.updateSupplyCurrentLimit(DriveConstants.SUPPLY_LIMIT_AUTO);
  }

  /** Applies the drive supply limit once per teleop entry, before driving commands run. */
  public void applyTeleopDriveCurrentLimit() {
    drive.updateSupplyCurrentLimit(DriveConstants.SUPPLY_LIMIT_TELEOP);
  }

  // Suppress driver translation/rotation while ReefAlign is active or Superstructure locks drive.
  private boolean driveInputLocked() {
    return superstructure.isDriveLocked()
      || superstructure.gamepad().leftBumper().getAsBoolean()
      || superstructure.gamepad().rightBumper().getAsBoolean();
  }

  /** Applies all four preload selections at the start of each autonomous period. */
  public void initializeSpinIndexerPreload() {
    if (simulationCorals != null) simulationCorals.resetReef();
    applyPreloadSelections();
  }

  /** Direct-to-teleop practice starts initialize once; auto-to-teleop never overwrites inventory. */
  public void initializeCoralPreloadIfNeeded() {
    if (!spinIndexerPreloadInitialized) {
      if (simulationCorals != null) simulationCorals.resetReef();
      applyPreloadSelections();
    }
  }

  private void applyPreloadSelections() {
    CoralState slot1 = selectedPreloadState(0);
    CoralState slot2 = selectedPreloadState(1);
    CoralState slot3 = selectedPreloadState(2);
    CoralState held = gripperPreloadChooser.getSelected();
    superstructure.applyPreload(slot1, slot2, slot3, held);
    if (simulationCorals != null) {
      simulationCorals.loadRobot(slot1, slot2, slot3, held);
    }
    spinIndexerPreloadInitialized = true;
    Logger.recordOutput("SpinIndexer/Preload/Slot1", slot1.name());
    Logger.recordOutput("SpinIndexer/Preload/Slot2", slot2.name());
    Logger.recordOutput("SpinIndexer/Preload/Slot3", slot3.name());
    Logger.recordOutput("EndEffector/Preload/Applied", held.name());
  }

  private CoralState selectedPreloadState(int slot) {
    return spinIndexerPreloadChoosers.get(slot).getSelected();
  }

  public void periodic() {
    MechanismVisualization.log(
        elevator.getHeight(), elevator.getPivotAngle(), spinIndexer.getPosition());
    if (simulationVision != null) {
      // Publish frames once per cycle for the next Vision periodic; never use corrected pose here.
      Pose2d truth = drive.getSimulationPose();
      simulationVision.update(truth);
      simulationCorals.update();
      Logger.recordOutput("Sim/RobotPose", truth);
      Logger.recordOutput("Sim/LocalizationErrorMeters",
          truth.getTranslation().getDistance(drive.getPose().getTranslation()));
    }
    Logger.recordOutput("Robot/Pose", drive.getPose());
    reefTargetColors.periodic();
    superstructure.periodic();
  }

  private VisionIO[] createVisionIO(List<VisionConstants.CameraConfig> configs) {
    VisionIO[] io = new VisionIO[configs.size()];
    for (int i = 0; i < io.length; i++) {
      if (simulationVision != null) {
        io[i] = new VisionIOPhotonVisionSim(configs.get(i), simulationVision);
      } else {
        // Replay restores logged observations without creating camera publishers/readers.
        io[i] = Constants.getMode() == Constants.Mode.REAL && VisionConstants.REAL_IO_CONFIGURED
            ? new VisionIOPhotonVision(configs.get(i)) : new VisionIO() {};
      }
    }
    return io;
  }

  private void resetSimulationPoseToOrigin() {
    Pose2d origin = new Pose2d();
    drive.resetSimulationPose(origin);
    simulationVision.resetRobotPose(origin);
    Logger.recordOutput("Sim/StartPoseError", "");
  }

  private static Command instant(String name, Runnable action) {
    return Command.noRequirements(coroutine -> action.run()).named(name);
  }

  private static final class EnumSelector<T extends Enum<T>> {
    private final StringEntry selected;
    private final T fallback;
    private final T[] options;

    EnumSelector(String path, T fallback, T[] options) {
      this.fallback = fallback;
      this.options = options.clone();
      selected =
          NetworkTableInstance.getDefault()
              .getStringTopic(path + "/Selected")
              .getEntry(fallback.name());
      NetworkTableInstance.getDefault()
          .getStringArrayTopic(path + "/Options")
          .publish()
          .set(Arrays.stream(options).map(Enum::name).toArray(String[]::new));
    }

    T getSelected() {
      String value = selected.get();
      return Arrays.stream(options)
          .filter(option -> option.name().equals(value))
          .findFirst()
          .orElse(fallback);
    }
  }

  /** Called before command scheduling; a stored alignment belongs to its previous alliance. */
  public void updateAlliance() {
    boolean currentRedAlliance = AllianceFlipUtil.shouldFlip();
    if (redAlliance == currentRedAlliance) return;
    redAlliance = currentRedAlliance;
    superstructure.observeAlignment(null);
    // Only disabled SIM practice teleports the world; REAL localization is never reset here.
    if (simulationVision != null && RobotState.isDisabled()) {
      resetSimulationPoseToOrigin();
      simulationCorals.resetReef();
    }
  }

  private static Drive createRealDrive() {
    if (!DriveConstants.REAL_IO_CONFIGURED) {
      return new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    }
    return new Drive(
      new GyroIOPigeon2(),
      new ModuleIOTalonFX(TunerConstants.FrontLeft),
      new ModuleIOTalonFX(TunerConstants.FrontRight),
      new ModuleIOTalonFX(TunerConstants.BackLeft),
      new ModuleIOTalonFX(TunerConstants.BackRight));
  }

  private static ModuleIO createSimModule(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    return new ModuleIOSim(constants);
  }

  private static Toggles createRealToggles() {
    if (!ToggleConstants.REAL_IO_CONFIGURED) {
      return new Toggles(new TogglesIO() {});
    }
    return new Toggles(new TogglesIOHardware());
  }

  private static SpinIndexer createRealSpinIndexer(Supplier<Distance> armHeight) {
    SpinIndexerIO motorIO = SpinIndexerConstants.REAL_IO_CONFIGURED
        ? new SpinIndexerIOTalonFX() : new SpinIndexerIO() {};
    return new SpinIndexer(motorIO, armHeight);
  }
  private static Elevator createRealElevator() {
    if (!ElevatorConstants.REAL_IO_CONFIGURED) {
      return new Elevator(new ElevatorIO() {});
    }
    return new Elevator(new ElevatorIOTalonFX());
  }

  private static EndEffector createRealEndEffector() {
    EndEffectorIO pneumaticIO = EndEffectorConstants.REAL_IO_CONFIGURED
        ? new EndEffectorIOHardware() : new EndEffectorIO() {};
    return new EndEffector(pneumaticIO);
  }

  private static CoralSensor createRealCoralSensor() {
    var gripper = CoralSensorConstants.GRIPPER;
    var indexer = CoralSensorConstants.INDEXER;
    boolean configured = CoralSensorConstants.REAL_IO_CONFIGURED;
    return new CoralSensor(gripper, indexer,
        configured ? new CANrangeIOPhoenix(gripper.canRangeId(), gripper.canBusName()) : new CANrangeIO() {},
        configured ? new ColorSensorIOREV(gripper.colorI2CPort()) : new ColorSensorIO() {},
        configured ? new CANrangeIOPhoenix(indexer.canRangeId(), indexer.canBusName()) : new CANrangeIO() {},
        configured ? new ColorSensorIOREV(indexer.colorI2CPort()) : new ColorSensorIO() {});
  }
}
