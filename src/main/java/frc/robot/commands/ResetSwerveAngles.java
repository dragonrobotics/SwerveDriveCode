package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class ResetSwerveAngles extends InstantCommand{
    private Swerve s_Swerve;
    public ResetSwerveAngles(Swerve m_swerve){
        s_Swerve = m_swerve;
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        for (SwerveModule module : s_Swerve.mSwerveMods) {
            module.setWheelAngle(0);
        }
    }
    
}
