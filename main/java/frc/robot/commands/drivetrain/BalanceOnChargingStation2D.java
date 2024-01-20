// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargingStation2D extends CommandBase {
  private final Drivetrain drivetrain;
  // private final ProfiledPIDController angleController = Drivetrain.getTunedRotationalPIDController();
  /**
   * Creates a new command that uses the tilt of the gyro to drive in the direction of the steepest incline.
   * @param drivetrain
   */
  public BalanceOnChargingStation2D(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // angleController.reset(drivetrain.getPose2d().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation3d gyroMeasurement = drivetrain.getGyroMeasurement();

    // Get the roll and pitch measurements from the gyro.
    double roll = gyroMeasurement.getX();
    double pitch = gyroMeasurement.getY();

    // Get the 'steepness' in the x (forward) and y (left) directions.
    double y = Math.sin(roll);
    double x = Math.sin(pitch);

    // If there isn't a significant tilt detected, assume the robot is in position and hold that position.
    if (x < .1 && y < .1) {
      drivetrain.moduleXConfiguration();
      return;
    }

    // Convert the x components of the roll and pitch into a Rotation2d that
    // points in the direction of the steepest upwards slope.
    Rotation2d steepestSlope = new Rotation2d(x, y);

    // Rotate the steepest slope by the error between the gyro measurement and the robot pose angle because if pose
    // is reset, the gyro and pose won't have the same angle.
    steepestSlope = steepestSlope.rotateBy(new Rotation2d(gyroMeasurement.getZ()).minus(drivetrain.getPose2d().getRotation()));

    // Calculate nearest square angle
    // double currentChassisAngle = drivetrain.getPose2d().getRotation().getRadians();
    // double nearestSquareChassisAngle = Math.floor((currentChassisAngle + Math.PI / 4.) / Math.PI/2.) * Math.PI/2.;

    // Controls how fast the robot moves when trying to balance.
    final double MOVEMENT_SPEED = 1; // in meters per second

    drivetrain.fromChassisSpeeds(new ChassisSpeeds(
      steepestSlope.getCos()*MOVEMENT_SPEED,
      steepestSlope.getSin()*MOVEMENT_SPEED,
      // angleController.calculate(currentChassisAngle, nearestSquareChassisAngle),
      0
    ));

    // // Show direction of steepest slope in SmartDashboard on the Modules (Field2d).
    // if (SmartDashboard.getBoolean("Modules Robot Relative", true)) {
    //   steepestSlope = steepestSlope.rotateBy(drivetrain.getPose2d().getRotation());
    // }

    // SmartDashboard.putNumberArray("Modules (Field2d)/Steepest Slope", new double[] {
    //   DrivetrainConstants.WHEEL_BASE_LENGTH_METERS,
    //   DrivetrainConstants.WHEEL_BASE_WIDTH_METERS,
    //   steepestSlope.getDegrees()
    // });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();

    // SmartDashboard.putNumberArray("Modules (Field2d)/Steepest Slope", new double[] {-10, -10, 0});
  }

  // We won't return true just in case the station is tipped by something else and we need to 
  // rebalance. It ensure we don't end the command too early.
  @Override
  public boolean isFinished() {
    return false;
  }
}
