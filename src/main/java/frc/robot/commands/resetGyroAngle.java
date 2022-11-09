package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class resetGyroAngle extends InstantCommand{
    private Swerve s_Swerve;
    public resetGyroAngle(Swerve m_swerve){
        s_Swerve = m_swerve;
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        for (SwerveModule mod : s_Swerve.mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        }
    }
    
}
