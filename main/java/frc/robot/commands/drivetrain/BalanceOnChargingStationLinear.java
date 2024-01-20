// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.LazyDashboard;

public class BalanceOnChargingStationLinear extends CommandBase {
  private final Drivetrain drivetrain;
  public static final double kP = 2; // proportion coefficient
  public static final double balanceTolerance = .04;

  public static double pitchOffset = 0;
  
  private int balanceCount = 0;
  
  public BalanceOnChargingStationLinear(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;

    SmartDashboard.putNumber("BalanceOnChargingStation/kP", kP);
    SmartDashboard.putNumber("BalanceOnChargingStation/balanceTolerance", balanceTolerance);
    LazyDashboard.addNumber("BalanceOnChargingStation/currentAngle", () -> drivetrain.getGyroMeasurement().getY() - pitchOffset);
  }

  private int signOfInitialAngle = 0;
  private boolean hasChangedSign = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balanceCount = 0;
    signOfInitialAngle = (int) Math.signum(drivetrain.getGyroMeasurement().getY() - pitchOffset);
    hasChangedSign = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation3d gyroMeasurement = drivetrain.getGyroMeasurement();

    double pitch = gyroMeasurement.getY() - pitchOffset; // Rotation around the y axis

    if (Math.abs(pitch) > balanceTolerance) {
      if (Math.signum(pitch) != signOfInitialAngle && signOfInitialAngle != 0) {
        hasChangedSign = true;
      }
      drivetrain.fromChassisSpeeds(
        new ChassisSpeeds(
          pitch * SmartDashboard.getNumber("BalanceOnChargingStation/kP", 0) * (hasChangedSign ? .8 : 1),
          0,
          0
        )
      );
    } else {
      drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    if (Math.abs(drivetrain.getGyroMeasurement().getY() - pitchOffset) < SmartDashboard.getNumber("BalanceOnChargingStation/balanceTolerance", balanceTolerance)) {
      balanceCount ++;
    } else {
      balanceCount = 0;
    }
    return balanceCount > 40;
  }

  public static void calibrateOffset() {
    pitchOffset = Drivetrain.getInstance().getGyroMeasurement().getY();
  }
}
