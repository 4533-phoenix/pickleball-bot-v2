package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public Angle upperServoAngle = Degrees.zero();
    public Angle lowerServoAngle = Degrees.zero();
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void upperServoAngle(Angle angle) {}

  public default void lowerServoAngle(Angle angle) {}
}
