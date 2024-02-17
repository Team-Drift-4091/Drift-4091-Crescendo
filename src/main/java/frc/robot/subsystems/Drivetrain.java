// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utility.BoringFalconSteerController;
import frc.robot.utility.PhotonVisionWrapper;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.drivetrains.SwerveDrivetrain;
import frc.team1891.common.drivetrains.swervemodules.DriveController;
import frc.team1891.common.drivetrains.swervemodules.FalconDriveController;
import frc.team1891.common.drivetrains.swervemodules.SwerveModule;
import frc.team1891.common.hardware.SimNavX;

import static frc.robot.utility.MirrorPoses.mirror;
import static frc.robot.Constants.DrivetrainConstants.*;


// Drive Train Class
public class Drivetrain extends SwerveDrivetrain {

  private static Drivetrain instance;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }


  private static final double TRANSLATIONAL_TOLERANCE = .02;
  private static final double ROTATIONAL_TOLERANCE = Math.toRadians(1);


  /**
   * Returns a new PID controller that can be used to control the position of the robot chassis in one axis.
   * @return a new {@link PIDController}
   */
  public static PIDController getTunedTranslationalPIDController() {
    PIDController controller = new PIDController(
      translationalP,
      translationalI,
      translationalD
    );
    controller.setTolerance(TRANSLATIONAL_TOLERANCE);
    return controller;
  }


  /**
   * Returns a new PID controller that can be used to control the angle of the robot chassis.
   * The output will be in radians per second, representing the desired angular velocity.
   * @return a new {@link ProfiledPIDController}
   */
  public static ProfiledPIDController getTunedRotationalPIDController() {
    ProfiledPIDController controller = new ProfiledPIDController(
      rotationalP,
      rotationalI,
      rotationalD,
      new TrapezoidProfile.Constraints(
        _config.chassisMaxAngularVelocityRadiansPerSecond(),
        _config.chassisMaxAngularAccelerationRadiansPerSecondSquared()
      )
    );
    controller.enableContinuousInput(0, 2*Math.PI);
    controller.setTolerance(ROTATIONAL_TOLERANCE);
    return controller;
  }


  /**
   * Returns a new PID controller that can be used to control the angle of the robot chassis.
   * The output will be between -1 and 1, and is meant to be fed to {@link Drivetrain#holonomicDrive(double, double, double, boolean)}.
   * @return a new {@link ProfiledPIDController}
   */
  public static ProfiledPIDController getTunedRotationalPIDControllerForHolonomicDrive() {
    ProfiledPIDController controller = new ProfiledPIDController(
      rotationalPForHolonomic,
      rotationalIForHolonomic,
      rotationalDForHolonomic,
      new TrapezoidProfile.Constraints(1, 1)
    );
    controller.enableContinuousInput(0, 2*Math.PI);
    controller.setTolerance(ROTATIONAL_TOLERANCE);
    return controller;
  }


  // Helpful object that holds information about the drivetrain.
  private static final DrivetrainConfig _config = new DrivetrainConfig(
    CHASSIS_MAX_VELOCITY,
    CHASSIS_MAX_ACCELERATION,
    CHASSIS_MAX_ANGULAR_VELOCITY,
    CHASSIS_MAX_ANGULAR_ACCELERATION,
    WHEEL_RADIUS_METERS,
    DRIVETRAIN_DRIVE_GEAR_RATIO,
    2048
  );


  private static final SimNavX _gyro = new SimNavX();

  private final PhotonVisionWrapper photonVision;



  // Objects to hold module related things, such as motors, and motor wrappers (drive and steer controllers)

  // Front Left
  private static final WPI_TalonFX frontLeftDriveMotor = new WPI_TalonFX(FrontLeft.DRIVE_CHANNEL);
  private static final DriveController frontLeftDriveController = new FalconDriveController(frontLeftDriveMotor, _config);
  private static final WPI_TalonFX frontLeftSteerMotor = new WPI_TalonFX(FrontLeft.STEER_CHANNEL);
  private static final WPI_CANCoder frontLeftEncoder = new WPI_CANCoder(FrontLeft.CANCODER_CHANNEL);
  private static final BoringFalconSteerController frontLeftSteerController = new BoringFalconSteerController(frontLeftSteerMotor, frontLeftEncoder, steerP, steerI, steerD);
  private static final SwerveModule frontLeft = new SwerveModule(frontLeftDriveController, frontLeftSteerController);

  // Front Right
  private static final WPI_TalonFX frontRightDriveMotor = new WPI_TalonFX(FrontRight.DRIVE_CHANNEL);
  private static final DriveController frontRightDriveController = new FalconDriveController(frontRightDriveMotor, _config);
  private static final WPI_TalonFX frontRightSteerMotor = new WPI_TalonFX(FrontRight.STEER_CHANNEL);
  private static final WPI_CANCoder frontRightEncoder = new WPI_CANCoder(FrontRight.CANCODER_CHANNEL);
  private static final BoringFalconSteerController frontRightSteerController = new BoringFalconSteerController(frontRightSteerMotor, frontRightEncoder, steerP, steerI, steerD);
  private static final SwerveModule frontRight = new SwerveModule(frontRightDriveController, frontRightSteerController);
  
  // Back Left
  private static final WPI_TalonFX backLeftDriveMotor = new WPI_TalonFX(BackLeft.DRIVE_CHANNEL);
  private static final DriveController backLeftDriveController = new FalconDriveController(backLeftDriveMotor, _config);
  private static final WPI_TalonFX backLeftSteerMotor = new WPI_TalonFX(BackLeft.STEER_CHANNEL);
  private static final WPI_CANCoder backLeftEncoder = new WPI_CANCoder(BackLeft.CANCODER_CHANNEL);
  private static final BoringFalconSteerController backLeftSteerController = new BoringFalconSteerController(backLeftSteerMotor, backLeftEncoder, steerP, steerI, steerD);
  private static final SwerveModule backLeft = new SwerveModule(backLeftDriveController, backLeftSteerController);

  //Back Right  
  private static final WPI_TalonFX backRightDriveMotor = new WPI_TalonFX(BackRight.DRIVE_CHANNEL);
  private static final DriveController backRightDriveController = new FalconDriveController(backRightDriveMotor, _config);
  private static final WPI_TalonFX backRightSteerMotor = new WPI_TalonFX(BackRight.STEER_CHANNEL);
  private static final WPI_CANCoder backRightEncoder = new WPI_CANCoder(BackRight.CANCODER_CHANNEL);
  private static final BoringFalconSteerController backRightSteerController = new BoringFalconSteerController(backRightSteerMotor, backRightEncoder, steerP, steerI, steerD);
  private static final SwerveModule backRight = new SwerveModule(backRightDriveController, backRightSteerController);



  // Drive Train
  private Drivetrain() {

    // Call super to initialize this Drivetrain as a SwerveDrivetrain, allowing the super class to handle important features.
    super(
      _config, 
      WHEEL_BASE_WIDTH_METERS,
      WHEEL_BASE_LENGTH_METERS,
      _gyro,
      frontLeft,
      frontRight,
      backLeft,
      backRight,
      MODULE_MAX_VELOCITY
    );

    // Steer motors are configured internally by the BSF_FalconSteerController
    configDriveMotor(frontLeftDriveMotor);
    configDriveMotor(frontRightDriveMotor);
    configDriveMotor(backLeftDriveMotor);
    configDriveMotor(backRightDriveMotor);

    configCANCoder(frontLeftEncoder, FrontLeft.ENCODER_OFFSET_DEGREES);
    configCANCoder(frontRightEncoder, FrontRight.ENCODER_OFFSET_DEGREES);
    configCANCoder(backLeftEncoder, BackLeft.ENCODER_OFFSET_DEGREES);
    configCANCoder(backRightEncoder, BackRight.ENCODER_OFFSET_DEGREES);

    configSteerMotor(frontLeftSteerMotor);
    configSteerMotor(frontRightSteerMotor);
    configSteerMotor(backLeftSteerMotor);
    configSteerMotor(backRightSteerMotor);


    // TODO: There may be some issues because this isn't reset before poseEstimator is initialized (@BullBotsLib).
    gyro.calibrate();
    gyro.reset();

    photonVision = PhotonVisionWrapper.getInstance();

    if (Robot.isSimulation()) {
      LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", 10, () -> simSpeeds.vxMetersPerSecond);
      LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", 10, () -> simSpeeds.vyMetersPerSecond);
      LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", 10, () -> simSpeeds.omegaRadiansPerSecond);
    }
    
    
    configureSmartDashboard();

  }

  private static void configDriveMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.config_kP(0, driveP);
    motor.config_kI(0, driveI);
    motor.config_kD(0, driveD);
    motor.config_kF(0, driveF);
    // Not sure if this will need tuned, I think it should be faster than the chassis
    // max to make sure the expected chassis movement isn't limited in auto.
    motor.configClosedloopRamp(.4*CHASSIS_MAX_VELOCITY/CHASSIS_MAX_ACCELERATION);
  }

  private static void configCANCoder(WPI_CANCoder encoder, double offset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(offset);
  }

  private static void configSteerMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    
    motor.setInverted(true);
  }

  public void setNeutralMode(NeutralMode mode) {
    frontLeftDriveMotor.setNeutralMode(mode);
    frontRightDriveMotor.setNeutralMode(mode);
    backLeftDriveMotor.setNeutralMode(mode);
    backRightDriveMotor.setNeutralMode(mode);
    frontLeftSteerMotor.setNeutralMode(mode);
    frontRightSteerMotor.setNeutralMode(mode);
    backLeftSteerMotor.setNeutralMode(mode);
    backRightSteerMotor.setNeutralMode(mode);
  }


  @Override
  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond();
    ySpeed *= config.chassisMaxVelocityMetersPerSecond();
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond();

    fromChassisSpeeds(
      fieldRelative?
        // If the robot is on the red alliance, the field oriented drive needs to change directions.
        // (Robot.isBlueAlliance()?
        //   ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation())
        // :
        //   ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))))
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
      :
        new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
  }


  /**
   * Returns the measurement of the gyro as a {@link Rotation3d}.
   * 
   * This measurement will not change if the robot's pose is reset.
   * However it will if the gyro itself is reset.
   * @return gyro measurements
   */
  public Rotation3d getGyroMeasurement() {
    // Certain axes of the gyro are inverted compared to the conventional Rotation3d.
    // Rotation3d assumes counterclockwise about each axis is positive.
    return new Rotation3d(Units.degreesToRadians(-_gyro.getRoll()), Units.degreesToRadians(_gyro.getPitch()), Units.degreesToRadians(-_gyro.getYaw()));
  }


  public boolean isGyroConnected() {
    return _gyro.isConnected();
  }


  @Override
  public void updateOdometry() {
    super.updateOdometry();

    // Handle vision processing and pose estimation
    if (Robot.isReal()) {
      Optional<EstimatedRobotPose> result =
        photonVision.getEstimatedGlobalPose(getPose2d());
    
      if (result.isPresent()){
        EstimatedRobotPose camPose = result.get();
        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        if (SmartDashboard.getBoolean("showPhotonEstimate", false)) {
          field.getObject("photonEstimate").setPose(camPose.estimatedPose.toPose2d());
        }
      } else {
        if (SmartDashboard.getBoolean("showPhotonEstimate", false)) {
          // move it way off the screen to make it disappear
          field.getObject("photonEstimate").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }
      }
    }
  }

  // @Override
  // public void periodic() {
  //   updateOdometry();
  //   field.setRobotPose(getPose2d());
  // }

  private ChassisSpeeds simSpeeds = new ChassisSpeeds();


  @Override
  public void simulationPeriodic() {
    // Calculate the theoretical chassis speed based on the desired position of the modules
    simSpeeds = kinematics.toChassisSpeeds(
      frontLeft.getDesiredSwerveModuleState(),
      frontRight.getDesiredSwerveModuleState(),
      backLeft.getDesiredSwerveModuleState(),
      backRight.getDesiredSwerveModuleState()
    );

    // update pose estimation and gyro readings according to theoretical chassis speeds
    _gyro.setRadians(_gyro.getRadians() - simSpeeds.omegaRadiansPerSecond * .02);
    Pose2d newPose = poseEstimator.getEstimatedPosition().plus(
      new Transform2d(
        new Translation2d(
          simSpeeds.vxMetersPerSecond * .02,
          simSpeeds.vyMetersPerSecond * .02
        ),
        new Rotation2d()
      )
    );

    poseEstimator.resetPosition(_gyro.getRotation2d(), getSwerveModulePositions(), newPose);
  }

  
  @Override
  protected void configureSmartDashboard() {
    super.configureSmartDashboard();
    LazyDashboard.addBoolean("Drivetrain/isGyroConnected", 100, this::isGyroConnected);
    
    SmartDashboard.putBoolean("Drivetrain/showPhotonEstimate", true);
  }
}
