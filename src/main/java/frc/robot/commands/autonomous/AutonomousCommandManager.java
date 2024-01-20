// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.Shoot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.trajectory.HolonomicTrajectoryCommandGenerator;

/**
 * Helper class to hold our autonomous commands.
 * 
 * Many of our autonomous commands will be built by chaining other commands together in sequence and parallel.
 * 
 * All our commands will be posted as options on {@link SmartDashboard} using a {@link SendableChooser}.
 */
public class AutonomousCommandManager {
    // This is private because we don't need to create any objects of this class; everything is static
    private AutonomousCommandManager() {}

    // This object appears on SmartDashboard, allowing us to select which autonomous routine we want, while the robot is on.
    private static SendableChooser<Command> commandChooser = new SendableChooser<>();

    /**
     * Loads the sendable chooser with all the autonomous options.
     */
    public static void load() {
        // RotatingHolonomicDriveController.enableSmartDashboard(true);
        HolonomicTrajectoryCommandGenerator.setRotationalPID(DrivetrainConstants.rotationalP, DrivetrainConstants.rotationalI, DrivetrainConstants.rotationalD);
        HolonomicTrajectoryCommandGenerator.setTranslationalPID(DrivetrainConstants.translationalP, DrivetrainConstants.translationalI, DrivetrainConstants.translationalD);

        // Default do nothing to avoid issues
        commandChooser.setDefaultOption("Default - Do Nothing", null);

        commandChooser.addOption("Shoot High and Taxi - NO BUMP", 
            new SequentialCommandGroup(
                new Shoot(Claw.getInstance(), ScoringLevel.HIGH).withTimeout(.4),
                new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(.5, 0, 0)), Drivetrain.getInstance()).withTimeout(6) 
            )
        );

        commandChooser.addOption("Shoot High and Taxi - WITH BUMP", 
            new SequentialCommandGroup(
                new Shoot(Claw.getInstance(), ScoringLevel.HIGH).withTimeout(.4),
                new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(.75, 0, 0)), Drivetrain.getInstance()).withTimeout(6) 
            )
        );

        commandChooser.addOption("Shoot High and Charge", 
            new SequentialCommandGroup(
                new Shoot(Claw.getInstance(), ScoringLevel.HIGH).withTimeout(.4),
                ChargeStation.autoChargeStation(Drivetrain.getInstance())
            )
        );

        commandChooser.addOption("Shoot High, Taxi, and Charge", 
            new SequentialCommandGroup(
                new Shoot(Claw.getInstance(), ScoringLevel.HIGH).withTimeout(.4),
                ChargeStation.autoChargeStationWithTaxi(Drivetrain.getInstance())
            )
        );

        commandChooser.addOption("Shoot Mid, Taxi, and Charge", 
            new SequentialCommandGroup(
                new Shoot(Claw.getInstance(), ScoringLevel.MID).withTimeout(.4),
                ChargeStation.autoChargeStationWithTaxi(Drivetrain.getInstance())
            )
        );

        SmartDashboard.putData("Autonomous Chooser", commandChooser);
    }

    /**
     * Gets the autonomous routine selected by the sendable chooser, and returns the correct command according to the alliance color.
     */
    public static Command getSelected() {
        // SmartDashboard.putBoolean("Autonomous Finished", false);
        // return commandChooser.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
        return commandChooser.getSelected();
    }
}