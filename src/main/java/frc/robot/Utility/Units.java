// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

// Utility class for unit conversions
public class Units {
    public static double ticksToDegrees(double ticks, double gearRatio, double ticksPerRev) {
        return (ticks * gearRatio * (1 / ticksPerRev)) * 360;
    }

    public static double degreesToTicks(double degrees, double gearRatio, double ticksPerRev) {
        return (degrees * (ticksPerRev / gearRatio)) / 360;
    }

    public static double ticksToMeters(double ticks, double gearRatio, double ticksPerRev, double metersPerRev) {
        return (ticks * gearRatio / ticksPerRev) * metersPerRev;
    }

    public static double metersToTicks(double meters, double gearRatio, double ticksPerRev, double metersPerRev) {
        return (meters / metersPerRev) * ticksPerRev / gearRatio;
    }

    public static double inchesToMeters(double inches) {
        return inches / 39.97;
    }
}
