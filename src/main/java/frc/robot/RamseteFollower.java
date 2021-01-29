// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/** Add your docs here. */
public class RamseteFollower {
    private final Timer m_timer = new Timer();
    private final boolean m_usePID;
    private final Trajectory m_trajectory;
    private final Pose2d m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final DifferentialDriveWheelSpeeds m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
  
    public RamseteFollower(
        Trajectory trajectory,
        Pose2d pose,
        RamseteController controller,
        SimpleMotorFeedforward feedforward,
        DifferentialDriveKinematics kinematics,
        DifferentialDriveWheelSpeeds wheelSpeeds,
        PIDController leftController,
        PIDController rightController) {
      m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
      m_pose = pose;
      m_follower = requireNonNullParam(controller, "controller", "RamseteCommand");
      m_feedforward = feedforward;
      m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
      m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "RamseteCommand");
      m_leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
      m_rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");
  
      m_usePID = true;
    }

    public void initialize() {
      m_prevTime = -1;
      var initialState = m_trajectory.sample(0);
      m_prevSpeeds =
          m_kinematics.toWheelSpeeds(
              new ChassisSpeeds(
                  initialState.velocityMetersPerSecond,
                  0,
                  initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
      m_timer.reset();
      m_timer.start();
      if (m_usePID) {
        m_leftController.reset();
        m_rightController.reset();
      }
    }
  
    public void execute(Spark motorLeft, Spark motorRight, DifferentialDrive m_diffDrive) {
      double curTime = m_timer.get();
      double dt = curTime - m_prevTime;
  
      if (m_prevTime < 0) {
        tankDrivevolts(0.0, 0.0, motorLeft, motorRight, m_diffDrive);
        m_prevTime = curTime;
        return;
      }
  
      var targetWheelSpeeds =
          m_kinematics.toWheelSpeeds(
              m_follower.calculate(m_pose, m_trajectory.sample(curTime)));
  
      var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
      var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

      System.out.println("Left Speed Set: " + leftSpeedSetpoint);
      System.out.println("Right Speed Set: " + rightSpeedSetpoint);
  
      double leftOutput;
      double rightOutput;
  
      if (m_usePID) {
        double leftFeedforward =
            m_feedforward.calculate(
                leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
  
        double rightFeedforward =
            m_feedforward.calculate(
                rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);
  
        leftOutput =
            leftFeedforward
                + m_leftController.calculate(m_speeds.leftMetersPerSecond, leftSpeedSetpoint);
  
        rightOutput =
            rightFeedforward
                + m_rightController.calculate(
                    m_speeds.rightMetersPerSecond, rightSpeedSetpoint);
      } else {
        leftOutput = leftSpeedSetpoint;
        rightOutput = rightSpeedSetpoint;
      }
  
      tankDrivevolts(leftOutput, rightOutput, motorLeft, motorRight, m_diffDrive);
      m_prevSpeeds = targetWheelSpeeds;
      m_prevTime = curTime;
    }
  
    public void end() {
      m_timer.stop();
    }
  
    public boolean isFinished() {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    private void tankDrivevolts(double leftVolts, double rightVolts, Spark motorLeft, Spark motorRight, DifferentialDrive m_diffDrive) {
        motorLeft.setVoltage(leftVolts);
        motorRight.setVoltage(-rightVolts);
        m_diffDrive.feed();
      }

  }
