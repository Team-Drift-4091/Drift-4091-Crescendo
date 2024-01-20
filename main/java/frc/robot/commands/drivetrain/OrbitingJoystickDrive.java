// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OrbitingJoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> orbitPoint;
  private final DoubleSupplier forward, strafe;

  private final ProfiledPIDController angleController;
  
  /**
   * Creates a command that drives according to the joystick in a field relative manner, while maintaining a heading towards the given orbit point.
   * @param drivetrain the drivetrian to control
   * @param orbitPoint the point to keep the robot facing
   * @param forward the forward input from the joystick
   * @param strafe the strafe input from the joystick
   */
  public OrbitingJoystickDrive(Drivetrain drivetrain, Supplier<Pose2d> orbitPoint, DoubleSupplier forward, DoubleSupplier strafe) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.orbitPoint = orbitPoint;
    this.forward = forward;
    this.strafe = strafe;

    // TODO: Tune PID
    // Theoretically this should be the same PID as fed to trajectories.
    // angleController = new ProfiledPIDController(1, 0, 0, 
    //   new TrapezoidProfile.Constraints(1,1)
    // );
    // angleController.enableContinuousInput(0, 2*Math.PI);
    angleController = Drivetrain.getTunedRotationalPIDControllerForHolonomicDrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.reset(drivetrain.getPose2d().getRotation().getRadians());
    angleController.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d target = orbitPoint.get();
    Pose2d currentPose = drivetrain.getPose2d();
    Rotation2d angleDeltaFromTarget = target.relativeTo(currentPose).getTranslation().getAngle();

    double twist = -angleController.calculate(angleDeltaFromTarget.getRadians());
    // double clampValue = .8;
    // double t = MathUtil.clamp(twist, -clampValue, clampValue);

    drivetrain.holonomicDrive(-forward.getAsDouble(), -strafe.getAsDouble(), twist, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
