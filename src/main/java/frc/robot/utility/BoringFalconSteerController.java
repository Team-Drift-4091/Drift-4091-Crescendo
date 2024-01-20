// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.team1891.common.drivetrains.swervemodules.SteerController;

/** Boring because it uses external PID. */
// This class was created because the FalconSteerController provided by BullBotsLib didn't work for us, since the 
// method setSelectedSensorPosition() didn't seem to be working for us.
public class BoringFalconSteerController implements SteerController {
    private final WPI_TalonFX steerMotor;
    private final CANCoder encoder;

    private final ProfiledPIDController pidController;

    public BoringFalconSteerController(WPI_TalonFX steerMotor, CANCoder encoder, double kP, double kI, double kD) {
        this.steerMotor = steerMotor;
        this.encoder = encoder;

        this.pidController = new ProfiledPIDController(kP, kI, kD, new Constraints(Math.PI*4, Math.PI*8));
        pidController.enableContinuousInput(0, 2*Math.PI);
    }

    @Override
    public void drive(SwerveModuleState state) {
        steerMotor.set(ControlMode.PercentOutput, pidController.calculate(this.getRadians(), state.angle.getRadians()));
    }

    @Override
    public void stop() {
        steerMotor.stopMotor();
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    @Override
    public double getRadians() {
        return Units.degreesToRadians(encoder.getAbsolutePosition());
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition();
    }
}
