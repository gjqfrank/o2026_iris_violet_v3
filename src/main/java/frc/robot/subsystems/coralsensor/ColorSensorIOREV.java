// Copyright (c) 2026 FRC 6907, The G.O.A.T
package frc.robot.subsystems.coralsensor;

import org.wpilib.hardware.bus.I2C;
import org.wpilib.system.Timer;

/** Checked I2C reads for the REV Color Sensor V3's APDS-9151, independent of pneumatic IO. */
public final class ColorSensorIOREV implements ColorSensorIO, AutoCloseable {
  // APDS-9151 datasheet pp. 16-22; settings also checked against REVLib 2026.0.5 source.
  private static final int ADDRESS = 0x52;
  private static final int PART_ID_REGISTER = 0x06;
  private static final int PART_ID = 0xC2;
  private static final int POWER_ON_FLAG = 0x20;
  private static final int COLOR_READY_FLAG = 0x08;
  private final I2C sensor;
  // Read the contiguous ID/status/RGB block; skip unused IR and proximity registers.
  private final byte[] frame = new byte[16];
  private boolean initialized;
  private boolean closed;

  /** Each instance must address its own isolated I2C bus; REV V3 has fixed address 0x52. */
  public ColorSensorIOREV(I2C.Port port) {
    sensor = new I2C(java.util.Objects.requireNonNull(port), ADDRESS);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    if (closed) return;
    // REVLib's raw getters ignore I2C aborts. Check the transaction before accepting zero values.
    if (sensor.read(PART_ID_REGISTER, frame.length, frame)
        || Byte.toUnsignedInt(frame[0]) != PART_ID) {
      initialized = false;
      // A failed transaction supplies no new measurement; retain the last values/timestamps.
      return;
    }
    int status = Byte.toUnsignedInt(frame[1]);
    if (!initialized || (status & POWER_ON_FLAG) != 0) {
      initialized = initializeSensor();
      return; // Wait for measurements made with the configured gain/resolution.
    }

    double timestamp = Timer.getTimestamp();
    if ((status & COLOR_READY_FLAG) != 0) {
      inputs.green = channelAt(7);
      inputs.blue = channelAt(10);
      inputs.red = channelAt(13);
      inputs.colorTimestampSeconds = timestamp;
    }
  }

  private boolean initializeSensor() {
    // TODO: Validate the 100 ms period, 18-bit resolution and 3x gain at each sensor mount.
    // Recalibrate brightness thresholds if these change. Presence is handled by CANrange.
    // Stop conversions during configuration. WPILib I2C returns true when a transfer aborts.
    return !sensor.write(0x00, 0x00)
        && !sensor.write(0x04, 0x22) // 18-bit color, 100 ms.
        && !sensor.write(0x05, 0x01) // 3x gain.
        && !sensor.write(0x00, 0x06); // RGB mode and light sensor enabled; proximity disabled.
  }

  private int channelAt(int offset) {
    return Byte.toUnsignedInt(frame[offset])
        | Byte.toUnsignedInt(frame[offset + 1]) << 8
        | Byte.toUnsignedInt(frame[offset + 2]) << 16;
  }

  @Override
  public void close() {
    if (closed) return;
    closed = true;
    initialized = false;
    sensor.close();
  }
}
