// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.utility.PhotonVisionWrapper;

// Simple command that interruptes the LEDDefaultCommand and checks if the limelight is connected.
// If it is, then it calls the execute of LEDDefaultCommand as normal.
// It will show the FAULT LEDMode if the limelight is disconnected
public class LEDLimelightFault extends CommandBase {
  private final LEDs leds;
  public LEDLimelightFault(LEDs leds) {
    addRequirements(leds);
    this.leds = leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (PhotonVisionWrapper.getInstance().photonCamera.isConnected()) {
      // leds.setMode(LEDMode.AUTONOMOUS);
      leds.getDefaultCommand().execute();
    } else {
      leds.setMode(LEDMode.FAULT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setMode(LEDMode.OFF);
    leds.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
