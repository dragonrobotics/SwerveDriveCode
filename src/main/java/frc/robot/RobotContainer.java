package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final Joystick joystick = new Joystick(0);
  private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick, 0, 1, 2);
  private Command fullAuto;
  public RobotContainer() {
    configureButtonBindings();
  }
  private void configureButtonBindings() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();
  eventMap.put("marker1", new PrintCommand("Passed marker 1"));

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      swerve::getPose, // Pose2d supplier
      swerve::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      swerve // The drive subsystem. Used to properly set the requirements of path following commands
  );

  fullAuto = autoBuilder.fullAuto(pathGroup);
    swerve.setDefaultCommand(teleopSwerve);
    JoystickButton reset_button = new JoystickButton(joystick, 1);
  }

  public Command getAutonomousCommand() {
    return fullAuto;
  }
}
