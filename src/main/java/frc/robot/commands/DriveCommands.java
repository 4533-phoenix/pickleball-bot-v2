// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {}

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive = forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command arcadeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds
          var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

          // Apply output
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
        },
        drive);
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runOpenLoop(voltage, voltage);
                  velocitySamples.add(drive.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /**
   * Drives the robot manually forward/backward while automatically rotating to face a known field
   * coordinate (the hub).
   */
  public static Command aimAtHub(Drive drive, DoubleSupplier fwdSupplier) {
    // The exact field coordinates of the hub (Update these X/Y meters for your specific game)
    Translation2d hubLocation = new Translation2d(4.6255178, 4.0346376);

    // PID Controller to calculate rotation speed based on angle error.
    PIDController thetaController = new PIDController(2.5, 0.0, 0.1);

    // Ensures the robot takes the shortest rotational path (e.g., passing through 180 degrees)
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          // Get the robot's current estimated pose on the field
          var currentPose = drive.getPose();

          // Calculate the target angle from the robot to the hub
          var targetAngle = hubLocation.minus(currentPose.getTranslation()).getAngle();

          // Calculate rotational speed using the PID controller
          double rotationSpeed =
              thetaController.calculate(
                  currentPose.getRotation().getRadians(), targetAngle.getRadians());

          // Calculate left/right wheel speeds using WPILib's Inverse Kinematics
          // We pass 'false' for squareInputs because the PID controller outputs linear corrections
          var speeds =
              DifferentialDrive.arcadeDriveIK(fwdSupplier.getAsDouble(), rotationSpeed, false);

          // Apply the calculated speeds to the drivetrain's closed-loop controllers
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
        },
        drive);
  }
}
