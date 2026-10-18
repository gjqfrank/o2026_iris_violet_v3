// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.energy;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Joules;
import static org.wpilib.units.Units.Watts;

import org.wpilib.util.Alert;

import frc.robot.constants.Constants;
import frc.robot.util.LogScope;
import java.util.Arrays;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Estimates the absolute motor load represented by registered supply-current reports.
 *
 * <p>This is not a measurement of total battery current, power, or energy: it excludes unreported
 * loads such as the roboRIO, radio, and any non-registered PDH channels. The current summary stays
 * in real logs at 10 Hz, while detailed channel and aggregate outputs are regenerated during replay
 * or emitted live only in tuning mode.
 */
public class BatteryLogger {
  private static final LogScope LOG = LogScope.of("Power/BatteryEstimate");
  private static final int SUMMARY_LOG_PERIOD_LOOPS = 5;
  private static final String SUMMARY_CURRENT_KEY = LOG.key("Summary/ReportedMotorLoadCurrentAmps");
  private static final String SUMMARY_POWER_KEY = LOG.key("Summary/EstimatedMotorLoadPowerWatts");
  private static final String SUMMARY_ENERGY_KEY =
      LOG.key("Summary/EstimatedMotorLoadEnergyJoules");

  private static final BatteryChannel[] CHANNELS = BatteryChannel.values();
  private static final BatteryAggregate[] AGGREGATES = BatteryAggregate.values();

  private static final BatteryLogger instance = new BatteryLogger();

  private static class BatteryEstimateInputs implements LoggableInputs {
    private double sourceVoltageVolts = 12.6;

    @Override
    public void toLog(LogTable table) {
      table.put("SourceVoltageVolts", sourceVoltageVolts);
    }

    @Override
    public void fromLog(LogTable table) {
      sourceVoltageVolts = table.get("SourceVoltageVolts", sourceVoltageVolts);
    }
  }

  // Per-tick current buffers, reset at end of periodicAfterScheduler.
  private final double[] channelCurrent = new double[CHANNELS.length];
  private final double[] aggregateCurrent = new double[AGGREGATES.length];
  private final BatteryEstimateInputs inputs = new BatteryEstimateInputs();

  // Cumulative energy since boot (matches legacy BatteryLogger semantics).
  private final double[] channelEnergy = new double[CHANNELS.length];
  private final double[] aggregateEnergy = new double[AGGREGATES.length];
  private double totalEnergy = 0.0;

  private boolean anyInvalidReportThisLoop = false;

  private final Alert invalidReportAlert =
      new Alert("invalid input","BatteryLogger received invalid supply current (NaN/Inf).", Alert.Level.MEDIUM);

  private BatteryLogger() {}

  public static BatteryLogger getInstance() {
    return instance;
  }

  public double getSourceVoltageVolts() {
    return inputs.sourceVoltageVolts;
  }

  /**
   * Captures the voltage source for this loop before subsystem periodic methods report currents.
   *
   * <p>The input is logged so replay can regenerate detailed power and energy diagnostics with the
   * voltage that was available on the real robot, rather than the desktop default battery voltage.
   */
  public void periodicBeforeScheduler(double sourceVoltageVolts) {
    if (Double.isFinite(sourceVoltageVolts)) {
      inputs.sourceVoltageVolts = Math.max(0.0, sourceVoltageVolts);
    }
    Logger.processInputs(LOG.key("Inputs"), inputs);
  }

  /**
   * Records a channel's supply current for this robot loop. Non-finite values are dropped and raise
   * an alert; negatives are normalized via abs (matches legacy behavior). Missing reports for a
   * channel are treated as 0 A for this tick.
   */
  public void reportSupplyCurrent(BatteryChannel channel, double amps) {
    if (!Double.isFinite(amps)) {
      anyInvalidReportThisLoop = true;
      return;
    }
    channelCurrent[channel.ordinal()] = Math.abs(amps);
  }

  /** Aggregates channels, integrates energy, and writes the configured log detail level. */
  public void periodicAfterScheduler() {
    double voltage = inputs.sourceVoltageVolts;
    double dt = Constants.loopPeriodSecs;
    boolean logDetailed = shouldLogDetailed();

    Arrays.fill(aggregateCurrent, 0.0);
    double totalCurrent = 0.0;

    for (int i = 0; i < CHANNELS.length; i++) {
      BatteryChannel channel = CHANNELS[i];
      double current = channelCurrent[i];
      double power = current * voltage;
      double energy = power * dt;

      channelEnergy[i] += energy;
      totalCurrent += current;

      for (BatteryAggregate parent : channel.parents) {
        aggregateCurrent[parent.ordinal()] += current;
        aggregateEnergy[parent.ordinal()] += energy;
      }

      if (logDetailed) {
        Logger.recordOutput(channel.currentKey, current, Amps);
        Logger.recordOutput(channel.powerKey, power, Watts);
        Logger.recordOutput(channel.energyKey, channelEnergy[i], Joules);
      }
    }

    double totalPower = totalCurrent * voltage;
    totalEnergy += totalPower * dt;

    if (logDetailed) {
      logSummary(totalCurrent, totalPower);
      for (int i = 0; i < AGGREGATES.length; i++) {
        BatteryAggregate agg = AGGREGATES[i];
        double aggCurrent = aggregateCurrent[i];
        Logger.recordOutput(agg.currentKey, aggCurrent, Amps);
        Logger.recordOutput(agg.powerKey, aggCurrent * voltage, Watts);
        Logger.recordOutput(agg.energyKey, aggregateEnergy[i], Joules);
      }
    } else {
      double summaryCurrent = totalCurrent;
      double summaryPower = totalPower;
      Logger.runEveryN(SUMMARY_LOG_PERIOD_LOOPS, () -> logSummary(summaryCurrent, summaryPower));
    }

    invalidReportAlert.set(anyInvalidReportThisLoop);
    anyInvalidReportThisLoop = false;
    Arrays.fill(channelCurrent, 0.0);
  }

  private static boolean shouldLogDetailed() {
    // return Constants.tuningMode || Logger.hasReplaySource();
    return true;
  }

  private void logSummary(double totalCurrent, double totalPower) {
    Logger.recordOutput(SUMMARY_CURRENT_KEY, totalCurrent, Amps);
    Logger.recordOutput(SUMMARY_POWER_KEY, totalPower, Watts);
    Logger.recordOutput(SUMMARY_ENERGY_KEY, totalEnergy, Joules);
  }
}
