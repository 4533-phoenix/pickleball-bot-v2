package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public enum Goal {
    CLOSED,
    OPEN,
    LOAD,
    RELEASE
  }

  @AutoLogOutput private Goal goal = Goal.CLOSED;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    switch (goal) {
      case CLOSED -> {
        io.upperServoAngle(IndexerConstants.SEMI_CLOSED_ANGLE);
        io.lowerServoAngle(IndexerConstants.CLOSED_ANGLE);
      }
      case OPEN -> {
        io.upperServoAngle(IndexerConstants.OPEN_ANGLE);
        io.lowerServoAngle(IndexerConstants.OPEN_ANGLE);
      }
      case LOAD -> {
        io.upperServoAngle(IndexerConstants.OPEN_ANGLE);
        io.lowerServoAngle(IndexerConstants.CLOSED_ANGLE);
      }
      case RELEASE -> {
        io.upperServoAngle(IndexerConstants.SEMI_CLOSED_ANGLE);
        io.lowerServoAngle(IndexerConstants.OPEN_ANGLE);
      }
    }
  }

  public Command setGoalCommand(Goal targetGoal) {
    return this.runOnce(() -> setGoal(targetGoal));
  }

  public Command loadHeld() {
    return this.startEnd(() -> setGoal(Goal.LOAD), () -> setGoal(Goal.CLOSED));
  }

  public Command releaseHeld() {
    return this.startEnd(() -> setGoal(Goal.RELEASE), () -> setGoal(Goal.CLOSED));
  }

  public Command loadRelease() {
    return Commands.sequence(
            this.runOnce(() -> setGoal(Goal.LOAD)),
            Commands.waitSeconds(0.5),
            this.runOnce(() -> setGoal(Goal.RELEASE)),
            Commands.waitSeconds(0.5))
        .repeatedly()
        .finallyDo(() -> setGoal(Goal.CLOSED));
  }
}
