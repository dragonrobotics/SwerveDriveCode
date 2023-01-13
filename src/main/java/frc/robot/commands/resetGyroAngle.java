package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class resetGyroAngle extends CommandBase{
    private Swerve s_Swerve;
    public resetGyroAngle(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        for (SwerveModule module : s_Swerve.mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        }
    }
}
