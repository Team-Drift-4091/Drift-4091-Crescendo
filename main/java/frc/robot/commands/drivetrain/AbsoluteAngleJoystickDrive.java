// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.leds.LEDDefaultCommand;
import frc.robot.subsystems.Drivetrain;

public class AbsoluteAngleJoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Rotation2d> rotation;
  private Rotation2d previousTarget = new Rotation2d();
  private final DoubleSupplier forward, strafe;

  private final ProfiledPIDController angleController;

  /**
   * Creates a command that drives according to the joystick.
   * 
   * This differs from the normal {@link JoystickDrive} in that the angle is determined by an absolute, field relative, rotation supplier.
   * @param drivetrain
   * @param forward
   * @param strafe
   * @param rotation
   */
  public AbsoluteAngleJoystickDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, Supplier<Rotation2d> rotation) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = rotation;

    angleController = Drivetrain.getTunedRotationalPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.reset(drivetrain.getPose2d().getRotation().getRadians());
    LEDDefaultCommand.isDrivingWithAbsoluteAngle = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Rotation2d currentAngle = drivetrain.getPose2d().getRotation();
    Rotation2d targetAngle = rotation.get();
    targetAngle = (targetAngle == null) ? previousTarget : targetAngle;

    double twist = angleController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
    twist /= DrivetrainConstants.CHASSIS_MAX_ANGULAR_VELOCITY;
    
    double f = -forward.getAsDouble();
    double s = -strafe.getAsDouble();
    if (Math.abs(f) < .01 && Math.abs(s) < .01) {
      twist *= 0;
    }

    drivetrain.holonomicDrive(f, s, twist, true);

    previousTarget = targetAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !drivetrain.isGyroConnected();
  }
}
