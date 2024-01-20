// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveToCube;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawJoint;
import frc.robot.subsystems.Drivetrain;

public class DriveToAndIntakeCube extends ParallelRaceGroup {
  /**
   * Creates a command that turns towards, then drives at a cube.  After a delay, it will lower the intake and start spinning.
   * The command can end on it's own if the limit switch is triggered (WIP), or if no cube is reported as visible
   */
  public DriveToAndIntakeCube(Drivetrain drivetrain, Claw claw, ClawJoint clawJoint) {
    addCommands(
      new DriveToCube(drivetrain),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new IntakeFull(claw, clawJoint)
      )
    );
  }
}
