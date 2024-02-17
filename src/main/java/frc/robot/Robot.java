// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.LazyDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final Thread lazyDashboardThread = new Thread(() -> {
    LazyDashboard.updateAll();
  }, "Update Lazy Dashboard");

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Port forward ports connected to the camera.  This allows us to access them when connected
    // to the robot over USB.

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
  public void teleopPeriodic() {}

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
