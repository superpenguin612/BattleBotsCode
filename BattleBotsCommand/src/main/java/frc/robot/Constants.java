// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final class Drivetrain {
        public static final class CANIDs {
            public static final int LEFT_PRIMARY = 1;
            public static final int LEFT_SECONDARY = 2;
            public static final int RIGHT_PRIMARY = 3;
            public static final int RIGHT_SECONDARY = 4;
        }

        public static final boolean LEFT_IS_INVERTED = false;
        public static final boolean RIGHT_IS_INVERTED = true;

        public static final double ENCODER_DISTANCE_TO_METERS = 1.0 * Math.PI
                * Constants.Drivetrain.Geometry.WHEEL_DIAMETER_METERS / 8.68; // encoder counts per revolution * (2 * pi
                                                                              // *
                                                                              // wheel radius) / gear ratio

        public static final class Geometry {
            /** Distance between centers of right and left wheels on robot. */
            public static final double TRACK_WIDTH_METERS = 0.62;
            public static final double WHEEL_DIAMETER_METERS = 0.14;
        }
    }

    public static final class Teleop {
        public static final boolean IS_TANK_DRIVE = true;
    }
}
