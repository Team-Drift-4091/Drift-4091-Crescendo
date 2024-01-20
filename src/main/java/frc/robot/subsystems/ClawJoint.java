// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawJointConstants;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.hardware.WPI_CANSparkMax;

public class ClawJoint extends SubsystemBase {
  // Singleton structure
  private static ClawJoint instance = null;
  public static ClawJoint getInstance() {
    if (instance == null) {
      instance = new ClawJoint();
    }
    return instance;
  }

  private final WPI_CANSparkMax motor;
  private final SparkMaxAbsoluteEncoder encoder;
  private final SparkMaxPIDController pidController;

  private ClawJoint() {
    motor = new WPI_CANSparkMax(ClawJointConstants.MOTOR_ID, WPI_CANSparkMax.MotorType.kBrushed);
    
    // Configure the motor
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(WPI_CANSparkMax.IdleMode.kBrake);

    motor.setSmartCurrentLimit(35, 35);

    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setInverted(true);
    pidController = motor.getPIDController();

    // Give the PID controller access to the encoder position
    pidController.setFeedbackDevice(encoder);
    pidController.setPositionPIDWrappingEnabled(true);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingMaxInput(1);

    // Configure PID
    // Going UP
    pidController.setP(2.7, 0);
    pidController.setI(0, 0);
    pidController.setD(1.2, 0);
    pidController.setFF(0, 0);
    pidController.setOutputRange(-.3, .6, 0);

    // Going DOWN
    pidController.setP(1.5, 1);
    pidController.setI(0, 1);
    pidController.setD(1.6, 1);
    pidController.setFF(0, 1);
    pidController.setOutputRange(-.3, .6, 1);

    // TODO: Need to figure out units
    // pidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    // pidController.setSmartMotionMaxVelocity(Math.PI/8., 0);
    // pidController.setSmartMotion

    // Give a SmartDashboard output of the angle
    LazyDashboard.addNumber("ClawJoint/angleRadians", this::getAngleRotations); // LazyDashboard updates the SmartDashboard value periodically using the getAngleRadians() method
  }

  public void drive(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void setIdleMode(IdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  /**
   * Drives the claw joint to the target angle
   * @param rotations target angle
   */
  public void setAngle(double rotations, boolean isDown) {
    SmartDashboard.putNumber("ClawJoint/targetAngle", rotations);
    int pidSlot = isDown ? 1 : 0;
    // // TODO: Make sure the target angle is attainable before trying to move.
    // if ((ClawJointConstants.MIN_ANGLE - 1) <= rotations && rotations <= ClawJointConstants.MAX_ANGLE) {
      pidController.setReference(rotations, WPI_CANSparkMax.ControlType.kPosition, pidSlot);
    // }
  }

  /**
   * Returns the current angle of the claw joint in rotations
   * @return current angle in rotations
   */
  public double getAngleRotations() {
    return encoder.getPosition();
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {}
}
