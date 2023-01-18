// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static final class Drive {
        public static final int kLeftMotorOneId = 4;
        public static final int kLeftMotorTwoId = 5;
        public static final int kLeftMotorThreeId = 6;

        public static final int kRightMotorOneId = 1;
        public static final int kRightMotorTwoId = 2;
        public static final int kRightMotorThreeId = 3;

        public static final double kSpeedDivider = 0.5;
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
