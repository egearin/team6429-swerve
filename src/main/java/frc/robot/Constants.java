// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
                public static final int FRONT_LEFT_MODULE_DRIVE = 7;
                public static final int FRONT_LEFT_MODULE_STEER = 8;
                public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
        
                public static final int FRONT_RIGHT_MODULE_DRIVE = 5;
                public static final int FRONT_RIGHT_MODULE_STEER = 6;
                public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
        
                public static final int BACK_LEFT_MODULE_DRIVE = 3;
                public static final int BACK_LEFT_MODULE_STEER = 4;
                public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
        
                public static final int BACK_RIGHT_MODULE_DRIVE = 1;
                public static final int BACK_RIGHT_MODULE_STEER = 2;
                public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;
        
                public static final int PIGEON_IMU = 0;

                //
                public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
                public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
                public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
                public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

                public static final double TRACKWIDTH = Units.inchesToMeters(0.0);
                public static final double WHEELBASE = Units.inchesToMeters(0.0);

                public static final double SPEED_MULTIPLIER = 0.0;

                public static final double MAX_VOLTAGE = 0.0;

                public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                                SdsModuleConfigurations.MK4_L3.getDriveReduction() *
                                SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;

                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                                Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                // Front left
                                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Front right
                                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                                // Back left
                                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Back right
                                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

        }
}