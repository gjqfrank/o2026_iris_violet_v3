// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.Distance;
import org.wpilib.hardware.bus.I2C;

public final class CoralSensorConstants {
  public static final boolean REAL_IO_CONFIGURED = false; // TODO: Enable after verifying both CANranges, isolated color-sensor buses and all sensor parameters.
  public static final int COLOR_ADC_MAX = 0x3FFFF; // Fixed 18-bit RGB configuration in ColorSensorIOREV.
  /** Time the CANrange detected bit must persist, on both edges, before presence is accepted, s. */
  public static final double DETECTED_DEBOUNCE_SECONDS = 0.05; // TODO: Validate against the CANrange sample rate and measured noise.
  public static final double COLOR_CONFIRM_SECONDS = 0.15; // TODO: Continuous same-color duration across distinct RGB measurements before inventory confirmation, s; validate both sensor mounts at the 100 ms measurement period.

  /** Each instance has independent hardware, presence thresholds and color calibration. */
  public record SensorConfig(
      String name, int canRangeId, String canBusName, I2C.Port colorI2CPort,
      Distance presenceThreshold, Distance presenceHysteresis, double minSignalStrength,
      double blackBrightnessMax, double whiteBrightnessMin) {}

  public static final SensorConfig GRIPPER = new SensorConfig(
      "Gripper",
      -1, // TODO: Gripper CANrange CAN ID; distinct from the indexer CANrange on the same bus.
      "", // TODO: Gripper CANrange bus name; use "rio" or the configured CANivore name.
      null, // TODO: Gripper color sensor I2C port; only kMXP is supported by the direct REV IO.
      Meters.of(Double.NaN), // TODO: Gripper Coral presence threshold, m from the CANrange face; separate from the indexer threshold. Detection enters below threshold minus hysteresis.
      Meters.of(0.01), // TODO: Gripper distance hysteresis, m; absence requires exceeding threshold plus hysteresis. Verify against measured noise and mounting distance.
      2500, // TODO: Minimum gripper CANrange signal strength; Phoenix default. Tune with both BLACK and WHITE Coral and the empty background.
      -1, // TODO: Maximum raw R+G+B brightness classified as BLACK held Coral, ADC counts.
      -1); // TODO: Minimum raw R+G+B brightness classified as WHITE held Coral; exceed the BLACK limit, ADC counts.

  public static final SensorConfig INDEXER = new SensorConfig(
      "Indexer",
      -1, // TODO: Indexer CANrange CAN ID; distinct from the gripper CANrange on the same bus.
      "", // TODO: Indexer CANrange bus name; use "rio" or the configured CANivore name.
      null, // TODO: Indexer color sensor I2C port. Two REV V3 sensors share address 0x52; a mux/coprocessor needs its own IO. Onboard I2C is unsupported due to roboRIO lockups.
      Meters.of(Double.NaN), // TODO: Slot Coral presence threshold, m from the fixed CANrange face; measure at the shared SLOT_ANGLE_OFFSET observation position.
      Meters.of(0.01), // TODO: Indexer distance hysteresis, m; verify both slot holders and BLACK/WHITE Coral at the shared sensor offset.
      2500, // TODO: Minimum indexer CANrange signal strength; Phoenix default. Cover BLACK/WHITE Coral and empty slot material.
      -1, // TODO: Maximum raw R+G+B brightness classified as BLACK slot Coral, ADC counts.
      -1); // TODO: Minimum raw R+G+B brightness classified as WHITE slot Coral; exceed the BLACK limit, ADC counts.

  private CoralSensorConstants() {}
}
