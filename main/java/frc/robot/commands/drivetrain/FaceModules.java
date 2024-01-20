// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Extremely simple command that rotates all the modules to face the given direction.
 * We used this in autonomous to make sure the modules faced the right direction before driving.
 */
public class FaceModules extends CommandBase {
  private final Drivetrain drivetrain;
  private final Rotation2d moduleRotation;

  public FaceModules(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    moduleRotation = new Rotation2d();
  }

  
  public FaceModules(Drivetrain drivetrain, Rotation2d moduleRotation) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.moduleRotation = moduleRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setSwerveModuleStates(
      new SwerveModuleState[] {
        new SwerveModuleState(0, moduleRotation),
        new SwerveModuleState(0, moduleRotation),
        new SwerveModuleState(0, moduleRotation),
        new SwerveModuleState(0, moduleRotation)
      }
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final SwerveModuleState[] states = drivetrain.getSwerveModuleStates();

    for (final SwerveModuleState state : states) {
      if (!compareState(state, moduleRotation)) {
        return false;
      }
    }
    return true;
  }

  private boolean compareState(SwerveModuleState currentState, Rotation2d targetAngle) {
    return Math.abs(currentState.angle.getRadians() - targetAngle.getRadians()) < .1;
  }
}
