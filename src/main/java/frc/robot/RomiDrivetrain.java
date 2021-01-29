// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class RomiDrivetrain {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController gamepad = new XboxController(0);

  private final DifferentialDriveOdometry m_odometry;

  private final DifferentialDriveVoltageConstraint voltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
                                  Constants.kDriveKinematics,
                                  10
    );

  private final TrajectoryConfig config =
    new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.kDriveKinematics)
    .addConstraint(voltageConstraint);

  RamseteFollower follower;

  Trajectory exampleTrajectory;

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterM) / Constants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterM) / Constants.kCountsPerRevolution);
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_diffDrive.setRightSideInverted(true);

    /*
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(1.0, 0, new Rotation2d(0.5))
      ),
      config
    );
    */

    String myJson = "paths/Unnamed.wpilib.json";

    exampleTrajectory = new Trajectory();
  
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(myJson);
      exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("err", e.getStackTrace());
    }

    follower = new RamseteFollower(
      exampleTrajectory,
      getPose(),
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      getWheelSpeeds(),
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0)
    );

  }

  public void updateDrivetrain() {
    var xaxisSpeed = gamepad.getY(Hand.kLeft);
    var zaxisRotate = gamepad.getX(Hand.kRight);

    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);

    SmartDashboard.putNumber("LeftWheelM", getLeftDistanceM());
    SmartDashboard.putNumber("RightWheelM", getRightDistanceM());
  }

  public void updateAutonomous() {
    follower.execute(m_leftMotor, m_rightMotor, m_diffDrive);
  }

  public void initializeAuto() {
    follower.initialize();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getLeftDistanceM(), getRightDistanceM());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceM() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceM() {
    return m_rightEncoder.getDistance();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftDistanceM(), getRightDistanceM());
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
}
