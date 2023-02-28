// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {

    public static final class TalonFX {
        public static final double kEncoderResolution = 2048;

        public static final int kTimeoutMs = 30;
    }

    public static final class Sensors {
        public static final int kDrivetrainGyroId = 11;
        public static final int kClawGyroId = 15;

        public static final int kClawSonarPingChannel = 1;
        public static final int kClawSonarEchoChannel = 0;
    }

    public static final class Drive {
        public static final int kLeftMotorOneId = 4;
        public static final int kLeftMotorTwoId = 5;
        public static final int kLeftMotorThreeId = 6;

        public static final int kRightMotorOneId = 1;
        public static final int kRightMotorTwoId = 2;
        public static final int kRightMotorThreeId = 3;

        public static final double kRampRate = 0.5;
    }

    public static final class Turret {
        public static final int kTurretMotorId = 10;

        public static final double kF = 0.0;
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 5;

        public static final double kMotorToDriving = (1.0 / 50.0);

        public static final double kDrivingToTurret = (14.0 / 70.0);

        public static final double kMotorToTurret = kMotorToDriving * kDrivingToTurret;
    }

    public static final class Shoulder {
        public static final int kLeftArmMotorId = 7;
        public static final int kRightArmMotorId = 8;

        public static final double kMotorToArm = (1.0 / 100.0) * (1.0 / 3.0);
    }

    public static final class Telescope {
        public static final int kTelescopeMotorId = 9;

        public static final double kMetersPerRev = 0.0127;

        public static final double kMotorToTelescope = (1.0 / 10.0);
    }

    public static final class Wrist {
        public static final int kWristMotorId = 12;

        public static final double kMotorToWrist = (1.0 / 100.0);
    }

    public static final class Intake {
        public static final int kFrontIntakeMotorId = 13;
        public static final int kBackIntakeMotorId = 14;
    }

    public static final class Input {
        public static final int kDriverControllerId = 0;
        public static final int kOperatorJoystickId = 1;

        public static final int kJoystickLeftTopLeftButtonId = 5;
        public static final int kJoystickLeftTopMiddleButtonId = 6;
        public static final int kJoystickLeftTopRightButtonId = 7;

        public static final int kJoystickLeftBottomLeftButtonId = 10;
        public static final int kJoystickLeftBottomMiddleButtonId = 9;
        public static final int kJoystickLeftBottomRightButtonId = 8;

        public static final int kJoystickRightTopLeftButtonId = 13;
        public static final int kJoystickRightTopMiddleButtonId = 12;
        public static final int kJoystickRightTopRightButtonId = 11;

        public static final int kJoystickRightBottomLeftButtonId = 14;
        public static final int kJoystickRightBottomMiddleButtonId = 15;
        public static final int kJoystickRightBottomRightButtonId = 16;

    }
}
