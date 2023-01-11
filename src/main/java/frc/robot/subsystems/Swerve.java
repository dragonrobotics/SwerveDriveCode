package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    private final SwerveDriveOdometry swerveDriveOdometry;


    public Swerve() {
        swerveDriveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getPitch(), );
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        SmartDashboard.putNumber("ModAngle", 0);
        SmartDashboard.putNumber("ModSpeed", 0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states;
    }

    public void drive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw()
                        ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(360 - ypr[0]);
    }
    public Rotation2d getPitch() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(360 - ypr[1]);
    }
    public Rotation2d getRoll() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(360 - ypr[2]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("Pitch", getPitch().getDegrees());
        SmartDashboard.putNumber("Roll", getRoll().getDegrees());
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                    mod.getEncoder());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                    mod.getState().speedMetersPerSecond);
        }

    }
}