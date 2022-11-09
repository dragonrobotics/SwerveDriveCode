package frc.lib.util;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final double angleOffset;
    public final boolean angleSensorInverted;
    public final boolean motorInverted;
    public final boolean angleInverted;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset, boolean angleSensorInverted, boolean motorInverted, boolean angleInverted) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleOffset = angleOffset;
        this.angleSensorInverted = angleSensorInverted;
        this.motorInverted = motorInverted;
        this.angleInverted = angleInverted;
    }
}
