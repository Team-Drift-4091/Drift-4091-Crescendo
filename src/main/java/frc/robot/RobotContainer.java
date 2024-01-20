// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToAndIntakeCube;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.*;
import frc.robot.commands.clawjoint.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.leds.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.utility.MirrorPoses;
import frc.robot.utility.LEDString.LEDPatterns;
import frc.team1891.common.control.AxisTrigger; 
import frc.team1891.common.control.POVTrigger;
import frc.team1891.common.control.POVTrigger.POV;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final ClawJoint clawJoint = ClawJoint.getInstance();
  private final Claw claw = Claw.getInstance();
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
  
  private final Trigger cancelAlignment = new AxisTrigger(ps4Controller, PS4Controller.Axis.kRightX.value, .05).or(
    () -> secondController.getPOV() == 270 || secondController.getPOV() == 90
  );
  private final Trigger leftXTrigger = new AxisTrigger(ps4Controller, PS4Controller.Axis.kLeftX.value, .05);
  private final Trigger leftYTrigger = new AxisTrigger(ps4Controller, PS4Controller.Axis.kLeftY.value, .05);
  private final Trigger alignForward = new POVTrigger(ps4Controller, POV.NORTH).or(new POVTrigger(secondController, POV.NORTH)).and(cancelAlignment.negate());
  private final Trigger alignReverse = new POVTrigger(ps4Controller, POV.SOUTH).or(new POVTrigger(secondController, POV.SOUTH)).and(cancelAlignment.negate());
  
   //Triggers and button bindings; PS4
  private final Trigger resetOdometry = new JoystickButton(ps4Controller, PS4Controller.Button.kOptions.value);

  private final Trigger scoreLow = new JoystickButton(ps4Controller, PS4Controller.Button.kCross.value);
  private final Trigger scoreMid = new JoystickButton(ps4Controller, PS4Controller.Button.kCircle.value);
  private final Trigger scoreHigh = new JoystickButton(ps4Controller, PS4Controller.Button.kTriangle.value);
  private final Trigger kobeShot = new JoystickButton(ps4Controller, PS4Controller.Button.kSquare.value);
  private final Trigger runIntake = new JoystickButton(ps4Controller, PS4Controller.Button.kR1.value);

  private final Trigger driveToCube = new JoystickButton(ps4Controller, PS4Controller.Button.kR2.value);
  // private final Trigger alignToCubeNode = new JoystickButton(ps4Controller, PS4Controller.Button.kL2.value);

  private final Trigger deployIntake = new JoystickButton(ps4Controller, PS4Controller.Button.kL1.value);

  private final Trigger chargeStationBalance = new JoystickButton(ps4Controller, PS4Controller.Button.kL3.value);

  private final Trigger resetGyroPitch = new JoystickButton(ps4Controller, PS4Controller.Button.kR3.value);

  private final Trigger ledRainbow = new JoystickButton(secondController, XboxController.Button.kLeftBumper.value);

  private final Trigger lockModules = new JoystickButton(secondController, XboxController.Button.kX.value)
    .and(cancelAlignment.negate())
    .and(leftYTrigger.negate())
    .and(leftXTrigger.negate());

  private final Trigger stopClawJoint = new JoystickButton(secondController, XboxController.Button.kY.value);

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

    clawJoint.setDefaultCommand(
      new HomeClawPosition(clawJoint)
    );

    alignForward.onTrue(new AbsoluteAngleJoystickDrive(drivetrain, 
      () -> ps4Controller.getLeftY(), 
      () -> ps4Controller.getLeftX(), 
      () -> MirrorPoses.getForwardForAlliance()));

    alignReverse.onTrue(new AbsoluteAngleJoystickDrive(drivetrain, 
      () -> ps4Controller.getLeftY(), 
      () -> ps4Controller.getLeftX(), 
      () -> MirrorPoses.getForwardForAlliance().rotateBy(Rotation2d.fromDegrees(180))));

    cancelAlignment.whileTrue(
      new JoystickDrive(
        drivetrain,
        () -> ps4Controller.getLeftY(),
        () -> ps4Controller.getLeftX(),
        () -> ps4Controller.getRightX()
      )
    );

    resetOdometry.onTrue(new InstantCommand(() -> {
      System.out.println("reset gyro.");
      drivetrain.resetGyro();
    }));

    resetGyroPitch.onTrue(new InstantCommand(() -> BalanceOnChargingStationLinear.calibrateOffset()));

    scoreLow.onTrue(new ParallelRaceGroup(
      new Shoot(claw, ScoringLevel.HYBRID).withTimeout(.4),
      new CustomColor(leds, 0, 50, 0)));
    scoreMid.onTrue(new ParallelRaceGroup(
      new Shoot(claw, ScoringLevel.MID).withTimeout(.4),
      new CustomColor(leds, 0, 50, 0)));
    scoreHigh.onTrue(new ParallelRaceGroup(
      new Shoot(claw, ScoringLevel.HIGH).withTimeout(.4),
      new CustomColor(leds, 0, 50, 0)));
    kobeShot.onTrue(new ParallelRaceGroup(
      new Shoot(claw, .75).withTimeout(.5),
      new CustomColor(leds, 150, 75, 0)));

    driveToCube.whileTrue(new ParallelRaceGroup(
      new DriveToAndIntakeCube(drivetrain, claw, clawJoint),
      new LEDLimelightFault(leds)
    ));

    runIntake.whileTrue(new Intake(claw));
  
    deployIntake.whileTrue(ClawToAngle.intake(clawJoint));

    chargeStationBalance.whileTrue(new BalanceOnChargingStationLinear(drivetrain));

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
    stopClawJoint.onTrue(new RunCommand(() -> {
      clawJoint.stop();
      clawJoint.setIdleMode(IdleMode.kCoast);
      LEDDefaultCommand.clawHasEStopped = true;
    }, clawJoint) {
      public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
      };
    });
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
