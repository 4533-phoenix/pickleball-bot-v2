// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * Physical constants and hardware configurations for the shooter subsystem.
 *
 * <p>Includes data on the game piece, actuator geometry, gear ratios, and motor controller
 * settings.
 */
public final class FlywheelConstants {
  //  Hardware IDs
  /** CAN ID for the flywheel motor controller. */
  public static final int CAN_ID = 1;

  /** Whether the flywheel motor is inverted. */
  public static final boolean INVERTED = true;

  //  Flywheel constants
  /** Motor model for the flywheel (Neo Vortex driven by SparkFlex). */
  public static final DCMotor GEARBOX = DCMotor.getNEO(1);

  /** Gear reduction ratio for the flywheel (motor to wheel). */
  public static final double REDUCTION = 1.0;

  /** Moment of inertia of the flywheel assembly. */
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0021);

  /** Radius of the flywheel driving wheel. */
  public static final Distance WHEEL_RADIUS = Inches.of(2.0);

  /** Allowed velocity error before the shooter is considered "ready". */
  public static final AngularVelocity ANGULAR_TOLERANCE = RadiansPerSecond.of(15.0);

  /** Maximum current limit for the flywheel motor. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(30.0);

  //  Encoder conversion factors
  /** Converts motor rotations to mechanism radians. */
  public static final double FLYWHEEL_ENCODER_POSITION_FACTOR = (2.0 * Math.PI) / REDUCTION;

  /** Converts motor RPM to mechanism rad/s. */
  public static final double FLYWHEEL_ENCODER_VELOCITY_FACTOR =
      ((2.0 * Math.PI) / 60.0) / REDUCTION;

  // PID constants for flywheel velocity control
  /** Proportional gain for flywheel velocity control. */
  public static final double FLYWHEEL_KP = 0.0085;

  /** Integral gain for flywheel velocity control. */
  public static final double FLYWHEEL_KI = 0.0;

  /** Derivative gain for flywheel velocity control. */
  public static final double FLYWHEEL_KD = 0.0001;

  /** Static friction feedforward gain for the flywheel. */
  public static final double FLYWHEEL_KS = 0.0;

  /** Velocity feedforward gain for the flywheel. */
  public static final double FLYWHEEL_KV = 0.0175;

  /** Acceleration feedforward gain for the flywheel. */
  public static final double FLYWHEEL_KA = 0.0;
}
