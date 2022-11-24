// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static final class CANIDs {
            public static final int FRONT_LEFT = 1; // FIXME
            public static final int FRONT_RIGHT = 2; // FIXME
            public static final int BACK_LEFT = 3; // FIXME
            public static final int BACK_RIGHT = 4; // FIXME
        }

        public static final class Inversion {
            public static final boolean FRONT_LEFT = false; // FIXME
            public static final boolean FRONT_RIGHT = false; // FIXME
            public static final boolean BACK_LEFT = false; // FIXME
            public static final boolean BACK_RIGHT = false; // FIXME

        }

        public static final double ENCODER_DISTANCE_TO_METERS = 1.0; // FIXME
    }
}
