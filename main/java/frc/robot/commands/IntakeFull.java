// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.claw.Intake;
import frc.robot.commands.clawjoint.ClawToAngle;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawJoint;

public class IntakeFull extends ParallelRaceGroup {
  public IntakeFull(Claw claw, ClawJoint clawJoint) {
    addCommands(
      new ClawToAngle(clawJoint, 0),
      new SequentialCommandGroup(
        new WaitCommand(.5),
        new Intake(claw)
      )
    );
  }
}
