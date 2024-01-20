// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.Shoot;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.subsystems.*;

// Creates a command that drives to the target location using pose estimation, and then shoots at the target level.
public class DriveToAndScore extends SequentialCommandGroup {
  public DriveToAndScore(Drivetrain drivetrain, Claw claw, ClawJoint clawJoint, ScoringLevel scoringLevel) {
    // If the target is to score low, we can align to any node, otherwise we need to go specifically to a cube node.
    if (scoringLevel == ScoringLevel.HYBRID) {
      addCommands(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestNodeAlignment()));
    } else {
      addCommands(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestCubeNodeAlignment()));
    }

    addCommands(new Shoot(claw, scoringLevel));
  }
}
