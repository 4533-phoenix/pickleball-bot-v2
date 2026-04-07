// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.WritableTrigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's shooter mechanism.
 *
 * <p>Responsible for controlling the angular velocity of the flywheels and the position of the
 * adjustable hood to regulate launch angle and distance.
 */
public class Flywheel extends SubsystemBase {
  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  private final Debouncer flywheelReadyDebouncer = new Debouncer(0.15, DebounceType.kFalling);

  /** High-level goals for the shooter subsystem. */
  public enum Goal {
    /** Stop all movement and motors. */
    STOP,
    /** Active and tracking a target state. */
    RUNNING,
    /** Only when we are characterizing the flywheel. */
    CHARACTERIZATION
  }

  @AutoLogOutput private Goal goal = Goal.STOP;
  private AngularVelocity state = RadiansPerSecond.zero();

  private final WritableTrigger flywheelReadyTrigger;
  private final Trigger readyToShootTrigger;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param flywheelIO The abstraction layer for the flywheel hardware.
   */
  public Flywheel(FlywheelIO flywheelIO) {
    this.flywheelIO = flywheelIO;

    // Build triggers once
    flywheelReadyTrigger = new WritableTrigger();
    readyToShootTrigger = flywheelReadyTrigger.and(() -> goal == Goal.RUNNING);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate
                null, // Use default step voltage
                null, // Use default timeout
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> flywheelIO.runCharacterization(voltage),
                null, // Telemetry is handled by standard AdvantageKit logging in periodic()
                this));
  }

  /**
   * Sets the high-level goal for the shooter.
   *
   * @param goal The target goal.
   */
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Flywheel", flywheelInputs);

    // Calculate if flywheel is ready
    boolean isFlywheelReady =
        Math.abs(flywheelInputs.velocity.in(RadiansPerSecond) - state.in(RadiansPerSecond))
            < ANGULAR_TOLERANCE.in(RadiansPerSecond);
    flywheelReadyTrigger.set(flywheelReadyDebouncer.calculate(isFlywheelReady));

    switch (goal) {
      case STOP -> {
        flywheelIO.stop();
      }
      case RUNNING -> {
        flywheelIO.setAngularVelocity(state);
      }
      case CHARACTERIZATION -> {}
    }
  }

  /**
   * Sets the target state for aiming (flywheel speed and hood angle).
   *
   * @param state The target shooter state.
   */
  public void setState(AngularVelocity state) {
    if (state == null) {
      this.state = RadiansPerSecond.zero();
      DriverStation.reportWarning(
          "Attempted to set shooter state to null. Defaulting to safe state.", false);
    } else {
      this.state = state;
    }
  }

  /**
   * Checks if the shooter is ready to launch a game piece.
   *
   * @return True if the flywheels are spun up, the hood is in position, and the shooter is active.
   */
  public Trigger isShooterReady() {
    return readyToShootTrigger;
  }

  /**
   * Returns a trigger that is true when the flywheel velocity is within tolerance.
   *
   * @return The flywheel ready trigger.
   */
  public Trigger isFlywheelReady() {
    return flywheelReadyTrigger;
  }

  /**
   * Returns the flywheel velocity error in radians per second (target minus actual).
   *
   * @return The flywheel velocity error in radians per second.
   */
  public double getFlywheelErrorRadPerSec() {
    return state.in(RadiansPerSecond) - flywheelInputs.velocity.in(RadiansPerSecond);
  }

  /**
   * Safely stops the flywheels and retracts the hood.
   *
   * @return A command to stop the shooter.
   */
  public Command stop() {
    return this.runOnce(() -> setGoal(Goal.STOP));
  }

  /**
   * Runs the shooter while the command is held.
   *
   * @return A command to run the shooter while held.
   */
  public Command runHeld() {
    return this.startEnd(() -> setGoal(Goal.RUNNING), () -> setGoal(Goal.STOP));
  }

  /**
   * Sets the shooter goal to RUNNING.
   *
   * @return A command to start the shooter.
   */
  public Command run() {
    return this.runOnce(() -> setGoal(Goal.RUNNING));
  }

  /** Convenience method to set the running goal directly. */
  public void setRunning() {
    setGoal(Goal.RUNNING);
  }

  /** Convenience method to set the stop goal directly. */
  public void setStop() {
    setGoal(Goal.STOP);
  }

  /**
   * Returns a command to run a quasistatic SysId test.
   *
   * @param direction The direction to run the SysId test (forward or reverse).
   * @return A command to run a quasistatic SysId test.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setGoal(Goal.CHARACTERIZATION))
        .andThen(
            run(() -> flywheelIO.runCharacterization(Volts.zero()))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction)))
        .finallyDo(() -> setGoal(Goal.STOP));
  }

  /**
   * Returns a command to run a dynamic SysId test.
   *
   * @param direction The direction to run the SysId test (forward or reverse).
   * @return A command to run a dynamic SysId test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setGoal(Goal.CHARACTERIZATION))
        .andThen(
            run(() -> flywheelIO.runCharacterization(Volts.zero()))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction)))
        .finallyDo(() -> setGoal(Goal.STOP));
  }
}
