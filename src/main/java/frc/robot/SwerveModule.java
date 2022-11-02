package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SwerveModule {
    private final WPI_TalonSRX turnMotor;
    private final CANSparkMax driveMotor;
    private final SparkMaxPIDController drivePidController;
    private final SwerveModuleConstants mConstants;
    private double lastAngle = 0;
    private RelativeEncoder encoder;

    private double neoToMPS(double sensorVelocity, double wheelCircumference, double gearRatio) {
        double motorRPM = sensorVelocity * (600.0 / 2048.0) / gearRatio * wheelCircumference / 60;
        return motorRPM;
    }

    public static double encoderToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    public static double degreesToEncoder(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        encoder = driveMotor.getEncoder();
        encoder.setInverted(false);
        driveMotor.setFeedbackDevice(encoder);
    }

    private void configTurnMotor() {
        turnMotor.setSensorPhase(mConstants.angleInverted);
        turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        turnMotor.config_kP(0, Constants.Swerve.angleKP);
        turnMotor.config_kD(0, Constants.Swerve.angleKD);
        turnMotor.config_kI(0, Constants.Swerve.angleKI);
        turnMotor.config_kF(0, Constants.Swerve.angleKF);
    }

    private void configPidController() {
        drivePidController.setP(Constants.Swerve.driveKP);
        drivePidController.setI(Constants.Swerve.driveKI);
        drivePidController.setD(Constants.Swerve.driveKD);
    }

    public int moduleNumber;

    public SwerveModule(int moduleId, SwerveModuleConstants constants) {
        moduleNumber = moduleId;
        mConstants = constants;
        turnMotor = new WPI_TalonSRX(mConstants.angleMotorID);
        configDriveMotor();
        driveMotor = new CANSparkMax(mConstants.driveMotorID, MotorType.kBrushless);
        configTurnMotor();
        drivePidController = driveMotor.getPIDController();
        configPidController();
    }

    public void setDesiredState(SwerveModuleState swerveModuleState) {
        SwerveModuleState desiredState = CTREModuleState.optimize(swerveModuleState, getState().angle);
        double velocity = MPSToNeo(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference,
                Constants.Swerve.driveGearRatio);
        drivePidController.setReference(.001, CANSparkMax.ControlType.kVelocity);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        //turnMotor.set(ControlMode.Position, degreesToEncoder(angle, Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private double MPSToNeo(double speedMetersPerSecond, double wheelcircumference, double drivegearratio) {
        return speedMetersPerSecond*60*(drivegearratio*wheelcircumference)*(600.0 / 2048.0);
    }

    public SwerveModuleState getState() {
        double velocity = driveMotor.getEncoder().getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(turnMotor.getSelectedSensorPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public void setWheelAngle(int i) {
        turnMotor.setSelectedSensorPosition(i);
    }
}
