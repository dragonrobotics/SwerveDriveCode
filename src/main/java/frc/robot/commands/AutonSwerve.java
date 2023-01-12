package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutonSwerve extends CommandBase{
    private Swerve s_Swerve;
    public AutonSwerve(Swerve s_Swerve){
        this.s_Swerve= s_Swerve;
        this.addRequirements(s_Swerve);
    }
    
}
