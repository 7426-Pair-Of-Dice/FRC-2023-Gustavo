// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static final class Drive {
        public static final int kLeftMotorOneId = 4;
        public static final int kLeftMotorTwoId = 5;
        public static final int kLeftMotorThreeId = 6;

        public static final int kRightMotorOneId = 1;
        public static final int kRightMotorTwoId = 2;
        public static final int kRightMotorThreeId = 3;

        public static final double kSpeedDivider = 0.5;

        public static final double kMotorToDriveShaftGearRatio = 10.71;

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 2.70;
        public static final double kaVoltSecondsSquaredPerMeter = 1.99;

        public static final double kPDriveVel = 7;
        
        public static final double kTrackWidth = 0.63;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kEncoderVelocityConversationFactor = 0.008;

        public static final double kMaxVoltage = 10;

    }

    public static final class Sensors {
        public static final int kPigeonId = 11;
    }

    public static final class AutoBalanceCommand {
        public static final double kMaxPower = 0.09;
        public static final double kTolerance = 3.0;
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kTargetAngle = 0.0;
    }

    public static final class DriveStraight {
        public static final double kP = 0.05;

        public static final double kSpeedDivider = 0.8;
    }
}
