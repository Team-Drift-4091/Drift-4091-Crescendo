// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.leds.LEDDefaultCommand;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  protected final Drivetrain drivetrain;
  protected final DoubleSupplier forward, strafe, twist;
  /**
   * Creates a new command that drives the robot according to joystick inputs (Double Suppliers). This is field oriented.
   * @param drivetrain
   * @param forward
   * @param strafe
   * @param twist
   */
  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier twist) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.twist = twist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDDefaultCommand.isDrivingWithAbsoluteAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (drivetrain.isGyroConnected()) {
    drivetrain.holonomicDrive(-forward.getAsDouble(), -strafe.getAsDouble(), -twist.getAsDouble(), true); // negative is forward on the joystick; chassis left is positive while joystick right is positive.
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
