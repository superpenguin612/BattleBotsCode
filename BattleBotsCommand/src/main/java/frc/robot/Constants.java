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
    }

    public static final class Teleop {
        public static final boolean IS_TANK_DRIVE = true;
    }
}
