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

        public static final int kConeSonarPingChannel = 3;
        public static final int kConeSonarEchoChannel = 2;
    }

    public static final class Drive {
        public static final int kLeftMotorOneId = 4;
        public static final int kLeftMotorTwoId = 5;
        public static final int kLeftMotorThreeId = 6;

        public static final int kRightMotorOneId = 1;
        public static final int kRightMotorTwoId = 2;
        public static final int kRightMotorThreeId = 3;

        public static final double kRampRate = 0.8;

        public static final double kMotorToWheel = ((50.0 / 12.0) * (60.0 / 14.0)) / 4.0;

        public static final double kEncoderResolution = 42.0;

        public static final double kWheelDiameter = 6.0; // in inches
    }

    public static final class Turret {
        public static final int kTurretMotorId = 10;

        public static final double kF = 0.0;
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 5.0;

        public static final double kDeadband = 0.05;

        public static final double kMotionCruiseVelocity = 40000;
        public static final double kMotionAcceleration = 30000;
        public static final int kMotionSCurveStrength = 1;

        public static final double kForwardSoftLimit = 360;
        public static final double kReverseSoftLimit = -360;

        public static final double kMotorToTurret = (1.0 / 50.0) * (14.0 / 70.0);
    }

    public static final class Shoulder {
        public static final int kLeftArmMotorId = 7;
        public static final int kRightArmMotorId = 8;

        public static final double kF = 0.2;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kRampRate = 1.0;

        public static final double kDeadband = 0.05;

        public static final double kMotionCruiseVelocity = 40000;
        public static final double kMotionAcceleration = 20000;
        public static final int kMotionSCurveStrength = 1;

        public static final double kForwardSoftLimit = 100.0;
        public static final double kReverseSoftLimit = 0.0;

        public static final double kMotorToArm = (1.0 / 100.0) * (1.0 / 3.0);
    }

    /* public static final class Telescope {
        public static final int kTelescopeMotorId = 9;

        public static final double kF = 0.0;
        public static final double kP = 0.08;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kDeadband = 0.05;

        public static final double kMotionCruiseVelocity = 25000;
        public static final double kMotionAcceleration = 15000;

        public static final double kForwardSoftLimit = Units.inchesToMeters(12.0);
        public static final double kReverseSoftLimit = 0.0;

        public static final double kMetersPerRev = 0.0127;

        public static final double kMotorToTelescope = (1.0 / 10.0);
    } */

    public static final class Wrist {
        public static final int kWristMotorId = 12;

        public static final double kF = 0.2;
        public static final double kP = 0.08;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kDeadband = 0.05;

        public static final double kMotionCruiseVelocity = 40000;
        public static final double kMotionAcceleration = 20000;

        public static final double kForwardSoftLimit = 180.0;
        public static final double kReverseSoftLimit = 0.0;

        public static final double kMotorToWrist = (1.0 / 100.0);
    }

    public static final class Intake {
        public static final int kLeftIntakeMotorId = 14;
        public static final int kRightIntakeMotorId = 13;
    }

    public static final class Input {
        public static final int kDriverControllerId = 0;
        public static final int kOperatorJoystickId = 1;

        public static final int kJoystickTriggerButtonId = 1;
        public static final int kJoystickCenterLeftButtonId = 3;
        public static final int kJoystickCenterMiddleButtonId = 2;
        public static final int kJoystickCenterRightButtonId = 4;

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

    public static final class Limelight {
        public static final int kRetroReflectivePipeline = 0;
        public static final int kAprilTagPipeline = 1;
        public static final int kCameraPipeline = 2;
    }

    public static final class DriveStraightCommand {
        public static final double kP = 0.002;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class DriveToDistanceCommand {
        public static final double kMinCommand = 0.2;

        public static final double kPDist = 0.8;
        public static final double kIDist = 0.0;
        public static final double kDDist = 0.0;

        public static final double kPAngle = 0.002;
        public static final double kIAngle = 0.0;
        public static final double kDAngle = 0.0;
    }

    public static final class RotateToAngleCommand {
        public static final double kMinCommand = 0.05;

        public static final double kP = 0.013;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class TurretTrackingCommand {
        public static final double kP = 0.04;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class AutoBalanceCommand {
        public static final double kMaxPower = 0.12;
        public static final double kP = 0.08;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
}
