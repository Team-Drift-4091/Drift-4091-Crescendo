// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.PhotonVisionWrapper;

// This was never used or even tested, however it was originally intended to drive towards a cube if one was detected in the limelight
public class DriveToCube extends CommandBase {
  private final Drivetrain drivetrain;
  private PhotonTrackedTarget result;
  private final PIDController pidController = new PIDController(.1, .1, 0); // TODO: tune
  public DriveToCube(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cantSeeCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> potentialResult = PhotonVisionWrapper.getInstance().getCubeLocation();
    if (potentialResult.isPresent()) {
      result = potentialResult.get();
      
      double xOffset = result.getYaw();
      SmartDashboard.putNumber("DriveToCube/xOffset", xOffset); // TODO unecessary networking
      double driveSpeed = 0;
      if (Math.abs(xOffset) < 20) {
        driveSpeed = .3;
      }

      drivetrain.fromChassisSpeeds(new ChassisSpeeds(
        driveSpeed,
        0,
        pidController.calculate(xOffset, 0)
      ));
    } else {
      drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  private int cantSeeCount = 0;

  public boolean canSee() {
    return cantSeeCount < 20;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (PhotonVisionWrapper.getInstance().seesCube()) {
      cantSeeCount = 0;
    } else {
      cantSeeCount++;
    }
    return cantSeeCount > 40 || (result != null && result.getArea() > 100.); // TODO tune value for when object is picked up or use limit switch
  }
}
