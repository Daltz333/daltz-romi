// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/** Add your docs here. */
public class Constants {
    // drivetrain
    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterM = 0.07;

    // characterization\
    public static final double ksVolts = 0.733;
    public static final double kvVoltSecondsPerMeter = 7.55;
    public static final double kaVoltSecondsSquaredPerMeter = 0.019;

    public static final double kPDriveVel = 0.074;

    public static final double kTrackwidthMeters = 0.64946;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 0.1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
