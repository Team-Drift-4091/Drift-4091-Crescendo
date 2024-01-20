// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.BalanceOnChargingStationLinear;
import frc.robot.commands.drivetrain.FaceModules;
import frc.robot.subsystems.Drivetrain;

public class ChargeStation {
  /** Creates a new AutoChargeStation. */
  public static SequentialCommandGroup autoChargeStation(Drivetrain drivetrain) {
    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
      // Reset the gyro before doing anything
      new InstantCommand(() -> {
        drivetrain.resetGyro();
        BalanceOnChargingStationLinear.calibrateOffset();
      }, drivetrain),

      // Align wheels forward before we start moving
      new FaceModules(drivetrain).withTimeout(1), // with a timeout just in case it doesn't end properly

      // Drive forward until the gyro angle is steep enough
      new RunCommand(() -> {
        // drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
        drivetrain.holonomicDrive(.35, 0, 0, false); // drive forward at 35% speed
      }, drivetrain) {
        @Override
        public void end(boolean interrupted) {
          drivetrain.stop();
        }
        @Override
        public boolean isFinished() {
            return Math.abs(drivetrain.getGyroMeasurement().getY() - BalanceOnChargingStationLinear.pitchOffset) > BalanceOnChargingStationLinear.balanceTolerance; // is pitch steeper than .2 rad
        }
      },

      // Attempt to balance
      new BalanceOnChargingStationLinear(drivetrain) // attempt to balance until gyro reads close to 0 in pitch
    );

    return command;
  }

    // Creates a command that drives over the charge station, using the gyro, then drives back up and balances.
    public static SequentialCommandGroup autoChargeStationWithTaxi(Drivetrain drivetrain) {
      SequentialCommandGroup command = new SequentialCommandGroup();
      command.addCommands(
        // Reset the gyro before doing anything
        new InstantCommand(() -> {
          drivetrain.resetGyro();
          BalanceOnChargingStationLinear.calibrateOffset();
        }, drivetrain),
  
        // Align wheels forward before we start moving
        new FaceModules(drivetrain).withTimeout(1), // with a timeout just in case it doesn't end properly
  
        // Drive forward until the gyro angle is steep enough
        new RunCommand(() -> {
          // drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
          drivetrain.holonomicDrive(.4, 0, 0, false); // drive forward at 40% speed
        }, drivetrain) {
          @Override
          public void end(boolean interrupted) {
            // drivetrain.stop();
          }
          @Override
          public boolean isFinished() {
              return drivetrain.getGyroMeasurement().getY() - BalanceOnChargingStationLinear.pitchOffset > BalanceOnChargingStationLinear.balanceTolerance; // is pitch steeper than .35 rad
          }
        },

        // Drive forward until the gyro angle is steep enough facing downwards
        new RunCommand(() -> {
          // drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
          drivetrain.holonomicDrive(.4, 0, 0, false); // drive forward at 40% speed
        }, drivetrain) {
          @Override
          public void end(boolean interrupted) {
            // drivetrain.stop();
          }
          @Override
          public boolean isFinished() {
              return (drivetrain.getGyroMeasurement().getY() - BalanceOnChargingStationLinear.pitchOffset) < -BalanceOnChargingStationLinear.balanceTolerance;
          }
        },

        // Drive forward until the gyro angle is level
        new RunCommand(() -> {
          // drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
          drivetrain.holonomicDrive(.4, 0, 0, false); // drive forward at 40% speed
        }, drivetrain) {
          @Override
          public void end(boolean interrupted) {
            // drivetrain.stop();
          }
          @Override
          public boolean isFinished() {
              return Math.abs(drivetrain.getGyroMeasurement().getY() - BalanceOnChargingStationLinear.pitchOffset) < BalanceOnChargingStationLinear.balanceTolerance;
          }
        },

        new WaitCommand(.5),

        // Drive backward until the gyro angle is steep enough facing downwards
        new RunCommand(() -> {
          // drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
          drivetrain.holonomicDrive(-.35, 0, 0, false); // drive backward at 35% speed
        }, drivetrain) {
          @Override
          public void end(boolean interrupted) {
            // drivetrain.stop();
          }
          @Override
          public boolean isFinished() {
              return (drivetrain.getGyroMeasurement().getY() - BalanceOnChargingStationLinear.pitchOffset) < -BalanceOnChargingStationLinear.balanceTolerance; // is pitch steeper than .2 rad
          }
        },
  
        // Attempt to balance
        new BalanceOnChargingStationLinear(drivetrain) // attempt to balance until gyro reads close to 0 in pitch
      );
  
      return command;
    }
}
