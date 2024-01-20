// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.subsystems.Claw;

public class ShootWithDelay extends CommandBase {
  private final Claw claw;
  private final Timer timer;
  private final double motorSpeed;
  
  public ShootWithDelay(Claw claw, ScoringLevel scoringLevel) {
    this(claw, scoringLevel.getRequiredMotorSpeed());
  }

  public ShootWithDelay(Claw claw, double motorSpeed) {
    addRequirements(claw);
    this.claw = claw;
    timer = new Timer();
    this.motorSpeed = motorSpeed;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setShootingMotors(motorSpeed);
    if (timer.hasElapsed(.5)) {
      claw.setInnerMotors(.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
