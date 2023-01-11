package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SwerveModule {
    private final WPI_TalonSRX turnMotor;
    private final CANSparkMax driveMotor;
    private final SparkMaxPIDController drivePidController;
    private final SwerveModuleConstants mConstants;

    public double encoderToDegrees(double ticks, double gearRatio) {
        double degrees = ((ticks - mConstants.angleOffset) / 1024) * -360;
        if (mConstants.angleInverted)
            degrees = degrees * -1;
        return degrees;
    }

    public double degreesToEncoder(double degrees, double gearRatio) {
        double ticks = (-(degrees) / 360) * 1024;
        if (mConstants.angleInverted)
            ticks = ticks * -1;
        return ticks + mConstants.angleOffset;
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(mConstants.motorInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);
        //driveMotor.setClosedLoopRampRate(.5);
    }

    private void configTurnMotor() {
        turnMotor.setSensorPhase(mConstants.angleSensorInverted);
        turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        turnMotor.config_kP(0, Constants.Swerve.angleKP);
        turnMotor.config_kD(0, Constants.Swerve.angleKD);
        turnMotor.config_kI(0, Constants.Swerve.angleKI);
        turnMotor.config_kF(0, Constants.Swerve.angleKF);
        turnMotor.configClosedloopRamp(0);
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
        driveMotor = new CANSparkMax(mConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();
        configTurnMotor();
        drivePidController = driveMotor.getPIDController();
        configPidController();
    }

    public void setDesiredState(SwerveModuleState swerveModuleState) {
        if(swerveModuleState.speedMetersPerSecond < 0.1){
            swerveModuleState.speedMetersPerSecond = 0;
            swerveModuleState.angle = getState().angle;
        }
        SwerveModuleState desiredState = CTREModuleState.optimize(swerveModuleState, getState().angle);
        SmartDashboard.putNumber("Mod" + moduleNumber + " Target Angle", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Mod" + moduleNumber + " Target Speed", desiredState.speedMetersPerSecond);
        double velocity = MPSToNeo(desiredState.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference,
                Constants.Swerve.driveGearRatio);


        drivePidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);

        turnMotor.set(ControlMode.Position, degreesToEncoder(desiredState.angle.getDegrees(),
                Constants.Swerve.angleGearRatio));
    }

    private double MPSToNeo(double speedMetersPerSecond, double wheelcircumference, double drivegearratio) {
        return ((((speedMetersPerSecond * 1000 / wheelcircumference) / drivegearratio) * 60) / 8.5) * 1.64;
    }

    private double neoToMPS(double sensorVelocity, double wheelCircumference, double drivegearratio) {
        return ((sensorVelocity * 4) / 1.64 * 8.5) * drivegearratio * wheelCircumference / 1000;
    }

    public SwerveModuleState getState() {
        double velocity = neoToMPS(driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(),Constants.Swerve.wheelCircumference,
        Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(encoderToDegrees(turnMotor.getSelectedSensorPosition(), 0));
        return new SwerveModuleState(velocity, angle);
    }

    public void setWheelAngle(int i) {
        turnMotor.setSelectedSensorPosition(i);
    }

    public double getEncoder() {
        return turnMotor.getSelectedSensorPosition();
    }
}
