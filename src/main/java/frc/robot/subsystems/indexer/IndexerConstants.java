package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class IndexerConstants {
  public static final int UPPER_SERVO_CHANNEL = 0;
  public static final int LOWER_SERVO_CHANNEL = 1;

  public static final Angle CLOSED_ANGLE = Degrees.of(180.0);
  public static final Angle SEMI_CLOSED_ANGLE = Degrees.of(145.0);
  public static final Angle OPEN_ANGLE = Degrees.of(102.0);
}
