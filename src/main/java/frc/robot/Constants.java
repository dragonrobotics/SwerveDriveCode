
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0;

        /* Drivetrain Constants */
        // TODO: GET NUMBERS YOU DIPSHIT
        public static final double trackWidth = .6725;
        public static final double wheelBase = .59;
        public static final double wheelDiameter = .1;
        public static final double wheelCircumference = .33;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.67 / 1.0); // 6.86:1
        public static final double angleGearRatio = (1.2 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 10;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.00006;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.00017;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final boolean angleSensorInverted = true;
            public static final boolean motorInverted = true;
            public static final boolean angleInverted = true;
            public static final int angleMotorID = 16;
            public static final double angleOffset = -413;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    angleOffset, angleSensorInverted, motorInverted, angleInverted);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final boolean angleSensorInverted = false;
            public static final boolean motorInverted = false;
            public static final boolean angleInverted = false;
            public static final int angleMotorID = 13;
            public static final double angleOffset = -185;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    angleOffset, angleSensorInverted, motorInverted, angleInverted);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 9;
            public static final boolean angleSensorInverted = true;
            public static final boolean motorInverted = true;
            public static final boolean angleInverted = true;
            public static final int angleMotorID = 15;
            public static final double angleOffset = -240;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    angleOffset, angleSensorInverted, motorInverted, angleInverted);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final boolean angleSensorInverted = true;
            public static final boolean motorInverted = false;
            public static final boolean angleInverted = true;
            public static final int angleMotorID = 14;
            public static final double angleOffset = -133;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    angleOffset, angleSensorInverted, motorInverted, angleInverted);
        }

    }
}
