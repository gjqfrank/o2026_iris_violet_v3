// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import static frc.robot.subsystems.coralsensor.CoralSensorConstants.COLOR_CONFIRM_SECONDS;
import static frc.robot.subsystems.coralsensor.CoralSensorConstants.DETECTED_DEBOUNCE_SECONDS;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import frc.robot.subsystems.coralsensor.CoralSensorConstants.SensorConfig;
import frc.robot.subsystems.spinindexer.CoralState;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Owns both CANrange/RGB pairs; commands explicitly commit confirmed observations to inventory. */
public final class CoralSensor implements Mechanism {
  private final Pair indexer;
  private final Pair endEffector;

  public CoralSensor(SensorConfig endEffectorConfig, SensorConfig indexerConfig,
      CANrangeIO endEffectorRangeIO, ColorSensorIO endEffectorColorIO,
      CANrangeIO indexerRangeIO, ColorSensorIO indexerColorIO) {
    Scheduler.getDefault().addPeriodic(this::periodic);
    endEffector = new Pair(endEffectorConfig, endEffectorRangeIO, endEffectorColorIO);
    indexer = new Pair(indexerConfig, indexerRangeIO, indexerColorIO);
  }

  private void periodic() {
    indexer.update();
    endEffector.update();
  }

  /** An empty Optional means pending; CoralState.EMPTY means confirmed absence. */
  public Optional<CoralState> getEndEffectorObservation() {
    return Optional.ofNullable(endEffector.observation);
  }

  public Optional<CoralState> getIndexerObservation() {
    return Optional.ofNullable(indexer.observation);
  }

  public void resetEndEffectorObservation() {
    endEffector.resetObservation();
  }

  public void resetIndexerObservation() {
    indexer.resetObservation();
  }

  /** Only distinct measurements advance confirmation; repeated cached readings cannot confirm it. */
  private static final class Pair {
    private final String logPath;
    private final CANrangeIO rangeIO;
    private final ColorSensorIO colorIO;
    private final CoralSensorTuning tuning;
    private final Debouncer presenceStable = new Debouncer(DETECTED_DEBOUNCE_SECONDS);
    private final CANrangeIOInputsAutoLogged rangeInputs = new CANrangeIOInputsAutoLogged();
    private final ColorSensorIOInputsAutoLogged colorInputs = new ColorSensorIOInputsAutoLogged();
    private Boolean presenceCandidate;
    private boolean presenceConfirmed;
    private double lastRangeTimestamp = Double.NaN;
    private double lastColorTimestamp = Double.NaN;
    private double colorCandidateSince = Double.NaN;
    private CoralState colorCandidate;
    private CoralState observation;

    private Pair(SensorConfig config, CANrangeIO rangeIO, ColorSensorIO colorIO) {
      this.rangeIO = rangeIO;
      this.colorIO = colorIO;
      logPath = "CoralSensor/" + config.name();
      tuning = new CoralSensorTuning(logPath, config);
    }

    private void update() {
      tuning.update(hashCode(), rangeIO, this::resetObservation);
      rangeIO.updateInputs(rangeInputs);
      Logger.processInputs(logPath + "/CANrange", rangeInputs);
      updatePresence();
      boolean present = presenceConfirmed && Boolean.TRUE.equals(presenceCandidate);
      if (present) colorIO.updateInputs(colorInputs);
      Logger.processInputs(logPath + "/ColorSensor", colorInputs);
      if (present) updateColor();
      Logger.recordOutput(logPath + "/Observation", observation == null ? "UNKNOWN" : observation.name());
    }

    private void updatePresence() {
      double timestamp = rangeInputs.measurementTimestampSeconds;
      if (!Double.isFinite(timestamp) || timestamp <= lastRangeTimestamp) return;
      lastRangeTimestamp = timestamp;
      if (presenceCandidate == null || presenceCandidate != rangeInputs.detected) {
        presenceCandidate = rangeInputs.detected;
        presenceStable.calculate(false);
        resetColor();
      }
      // Debounce candidate stability, so both presence and absence wait, including startup.
      presenceConfirmed = presenceStable.calculate(true);
      if (!presenceConfirmed) observation = null;
      else if (!presenceCandidate) observation = CoralState.EMPTY;
    }

    private void updateColor() {
      double timestamp = colorInputs.colorTimestampSeconds;
      if (!Double.isFinite(timestamp) || timestamp <= lastColorTimestamp) return;
      lastColorTimestamp = timestamp;
      double brightness = (double) colorInputs.red + colorInputs.green + colorInputs.blue;
      CoralState color = brightness <= tuning.blackBrightnessMax.get() ? CoralState.BLACK
          : brightness >= tuning.whiteBrightnessMin.get() ? CoralState.WHITE : null;
      if (color != colorCandidate) {
        colorCandidate = color;
        colorCandidateSince = timestamp;
        observation = null;
      }
      // A later distinct sample is required even if the configured duration is zero.
      if (color != null && timestamp > colorCandidateSince
          && timestamp - colorCandidateSince >= COLOR_CONFIRM_SECONDS) observation = color;
    }

    private void resetObservation() {
      presenceCandidate = null;
      presenceConfirmed = false;
      presenceStable.calculate(false);
      lastRangeTimestamp = rangeInputs.measurementTimestampSeconds;
      resetColor();
    }

    private void resetColor() {
      colorCandidate = observation = null;
      colorCandidateSince = Double.NaN;
      lastColorTimestamp = colorInputs.colorTimestampSeconds;
    }
  }
}
