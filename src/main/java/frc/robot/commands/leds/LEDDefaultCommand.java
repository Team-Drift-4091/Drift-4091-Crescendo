// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;

public class LEDDefaultCommand extends CommandBase {
  private final LEDs leds;
  public LEDDefaultCommand(LEDs leds) {
    addRequirements(leds);
    this.leds = leds;
  }

  public static boolean isDrivingWithAbsoluteAngle = false;
  public static boolean clawHasEStopped = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!DriverStation.isDSAttached()) {
      leds.setMode(LEDMode.DISCONNECTED);
    } else if (!Drivetrain.getInstance().isGyroConnected() || clawHasEStopped) {
      leds.setMode(LEDMode.FAULT);
    } else if (DriverStation.isDisabled()) {
      if (AutonomousCommandManager.getSelected() == null) {
        leds.setMode(LEDMode.FAULT);
      }
      leds.setMode(LEDMode.DISABLED);
    } else if (DriverStation.isAutonomousEnabled()) {
      leds.setMode(LEDMode.AUTONOMOUS);
    } else if (DriverStation.isTeleopEnabled()) {
      if (isDrivingWithAbsoluteAngle) {
        leds.setMode(LEDMode.TELEOP_SPECIAL);
      } else {
        leds.setMode(LEDMode.TELEOP);
      }
    } else {
      leds.setMode(LEDMode.OFF);
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

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }
}
