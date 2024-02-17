// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.AutonomousCommandManager;

import frc.robot.commands.drivetrain.*;
import frc.robot.commands.leds.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.utility.LEDString.LEDPatterns;
import frc.team1891.common.control.AxisTrigger;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final LEDs leds = LEDs.getInstance();

  // controllers: PS4
  private final PS4Controller ps4Controller = new PS4Controller(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };

  private final XboxController secondController = new XboxController(1) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };
  
  private final Trigger cancelAlignment = new AxisTrigger(secondController, XboxController.Axis.kRightX.value, .05).or(
    () -> secondController.getPOV() == 270 || secondController.getPOV() == 90
  );
  private final Trigger leftXTrigger = new AxisTrigger(secondController, XboxController.Axis.kLeftX.value, .05);
  private final Trigger leftYTrigger = new AxisTrigger(secondController, XboxController.Axis.kLeftY.value, .05);
  //Triggers and button bindings; PS4


  private final Trigger ledRainbow = new JoystickButton(secondController, XboxController.Button.kLeftBumper.value);

  private final Trigger lockModules = new JoystickButton(secondController, XboxController.Button.kX.value)
    .and(cancelAlignment.negate())
    .and(leftYTrigger.negate())
    .and(leftXTrigger.negate());

  public RobotContainer() {
    // Connects the buttons and triggers to commands
    configureBindings();

    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    // Loads the autonomous chooser with all of the available autonomous routines.
    // I'm doing this on a seperate thread because loading trajectories can take a lot of time.
    Thread loadAutoThread = new Thread(() -> {
      long start = System.currentTimeMillis();
      AutonomousCommandManager.load();
      System.out.printf("AUTO: Autonomous commands loaded in %.3f seconds.\n", (System.currentTimeMillis() - start)/1000.);
    }, "AutonomousCommandManager.load();");
    loadAutoThread.run();
  }
  //xbox
  private void configureBindings() {
    // DEFAULT COMMANDS
    leds.setDefaultCommand(new LEDDefaultCommand(leds));

    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> ps4Controller.getLeftY(),
        () -> ps4Controller.getLeftX(),
        () -> ps4Controller.getRightX()
      )
    );

    cancelAlignment.whileTrue(
      new JoystickDrive(
        drivetrain,
        () -> ps4Controller.getLeftY(),
        () -> ps4Controller.getLeftX(),
        () -> ps4Controller.getRightX()
      )
    );



    
    ledRainbow.onTrue(new InstantCommand(() -> {
      leds.start();
      leds.setCustomPattern(LEDPatterns.RAINBOW);
    }, leds) {
      public void end(boolean interrupted) {
        leds.setMode(LEDMode.OFF);
        leds.stop();
      };
      public boolean isFinished() {
        return false;
      };
    }.withTimeout(5));

    lockModules.whileTrue(new RunCommand(() -> {
      drivetrain.moduleXConfiguration();
      leds.setMode(LEDMode.TELEOP_SPECIAL);
    }, drivetrain, leds) {
      public void initialize() {
        leds.start();
      };
      public void end(boolean interrupted) {
        leds.setMode(LEDMode.OFF);
        leds.stop();
      };
    });

    // Interrupt claw's default command - for emergencies so we can try to avoid frying the motor
    // This command has a special interrupt behavior, meaning you can't start the claw joint motor again no matter what you press.
    
  }

  // This method runs at the beginning of the match to determine what command runs in autonomous.
  public Command getAutonomousCommand() {
    // Post to SmartDashboard that the command has started
    SmartDashboard.putBoolean("Autonomous Finished", false);
    // We need this in order to avoid a crash when running the same command twice. In a match this would never happen
    // but it's necessary for testing.  We only need it because were adding the .andThen with the SmartDashboard output.
    CommandScheduler.getInstance().clearComposedCommands();
    // Returns the selected command from AutonomousCommandManager and appends a simple instant command that tells
    // SmartDashboard the command finished.
    Command autoCommand = AutonomousCommandManager.getSelected();
    if (autoCommand == null) {
      autoCommand = new InstantCommand();
    }
    return autoCommand.andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
  } 
}
