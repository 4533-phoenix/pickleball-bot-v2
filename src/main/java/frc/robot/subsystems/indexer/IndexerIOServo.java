package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class IndexerIOServo implements IndexerIO {
  private final Servo upperServo = new Servo(IndexerConstants.UPPER_SERVO_CHANNEL);
  private final Servo lowerServo = new Servo(IndexerConstants.LOWER_SERVO_CHANNEL);

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.upperServoAngle = Degrees.of(upperServo.getAngle());
    inputs.lowerServoAngle = Degrees.of(lowerServo.getAngle());
  }

  @Override
  public void upperServoAngle(Angle angle) {
    upperServo.setAngle(angle.in(Degrees));
  }

  @Override
  public void lowerServoAngle(Angle angle) {
    lowerServo.setAngle(angle.in(Degrees));
  }
}
