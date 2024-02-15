// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.LazyDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private TalonFX leftMotorOne;
  private TalonFX leftMotorTwo;
  private Joystick controller;

  private final Thread lazyDashboardThread = new Thread(() -> {
    LazyDashboard.updateAll();
  }, "Update Lazy Dashboard");

  @Override
  public void robotInit() {
    // This method is called once when the robot is first started up.
    // It is used to perform initialization tasks, such as setting up motor controllers, joysticks, and other hardware.
    // In this method, we instantiate the RobotContainer, which sets up button bindings and autonomous chooser on the dashboard.
    // We also initialize motor controllers (TalonFX) for driving motors and a joystick for controlling the robot.
    // Finally, we set the initial speed of the motors to 0 to ensure they start at a stopped state.
    m_robotContainer = new RobotContainer();
    leftMotorOne = new TalonFX(20);
    leftMotorTwo = new TalonFX(21);
    controller = new Joystick(0);
    leftMotorOne.set(0);
    leftMotorTwo.set(0);
}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    lazyDashboardThread.run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // In this method, we read the values of the left and right triggers on the controller, which control the intake and shooter motors respectively.
    // We calculate the motor speeds based on the trigger values, where the left trigger controls the intake and the right trigger controls the shooter.
    // Then, we set the motor speeds accordingly to achieve the desired movement.
    // Needs to be moved out of robot.java and organized into the subsystems
    double leftTrigger = controller.getRawAxis(2); // Reads the value of the left trigger
    double rightTrigger = controller.getRawAxis(3); // Reads the value of the right trigger

    double intakeSpeed = leftTrigger / 2.0; // Calculates intake motor speed (scaled to half for control)
    double shooterSpeed = rightTrigger; // Shooter motor speed directly controlled by the right trigger

    // Set motor speeds based on trigger values
    if (intakeSpeed == 0 && shooterSpeed > 0) { // If only the shooter is active
        leftMotorOne.set(shooterSpeed);
        leftMotorTwo.set(shooterSpeed);
    } else if (intakeSpeed > 0 && shooterSpeed == 0) { // If only the intake is active
        leftMotorOne.set(-intakeSpeed); // Assuming intake needs to spin in reverse direction
        leftMotorTwo.set(-intakeSpeed); // Assuming intake needs to spin in reverse direction
    } else { // If neither or both triggers are active
        leftMotorOne.set(0); // Stop both motors
        leftMotorTwo.set(0); // Stop both motors
    }
}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    Drivetrain.getInstance().setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void testPeriodic() {
    // I guess this doesn't get called normally? We need it for diagnostics
    Drivetrain.getInstance().periodic();
  }

  @Override
  public void testExit() {
    Drivetrain.getInstance().setNeutralMode(NeutralMode.Brake);
  }
}
