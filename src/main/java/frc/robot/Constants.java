// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {

    public static final class TalonFX {
        public static final double kEncoderResolution = 2048;

        public static final int kTimeoutMs = 30;
    }

    public static final class Drive {
        public static final int kLeftMotorOneId = 4;
        public static final int kLeftMotorTwoId = 5;
        public static final int kLeftMotorThreeId = 6;

        public static final int kRightMotorOneId = 1;
        public static final int kRightMotorTwoId = 2;
        public static final int kRightMotorThreeId = 3;

        public static final double kSpeedDivider = 0.5;
    }

    public static final class Turret {
        public static final int kTurretMotorId = 10;

        public static final double kTurretPercentOutput = 0.25;

        public static final int kPIDLoopId = 0;

        public static final double kP = 0.15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kMotorToDriving = (1 / 10);

        public static final double kDrivingToTurret = (70 / 14);

        public static final double kReverseSoftLimitOffset = 95000; // In ticks
        public static final double kForwardSoftLimitOffset = 98000; // In ticks
    }

    public static final class Arm {
        public static final int kLeftAngleMotorId = 7;
        public static final int kRightAngleMotorId = 8;

        public static final int kTelescopeMotorId = 9;

        public static final double kForwardLimitOffset = 80000; // In ticks
    }

    public static final class Input {
        public static final int kDriverControllerId = 0;
        public static final int kOperatorControllerId = 1;
    }
}
