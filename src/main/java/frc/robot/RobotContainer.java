package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ResetSwerveAngles;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final Joystick joystick = new Joystick(0);
  private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick, 0, 1, 2);
  private final ResetSwerveAngles resetSwerveAngles = new ResetSwerveAngles(swerve);

  public RobotContainer() {
    configureButtonBindings();
  }
  private void configureButtonBindings() {
    swerve.setDefaultCommand(teleopSwerve);
    JoystickButton reset_button = new JoystickButton(joystick, 10);
    reset_button.whenPressed(resetSwerveAngles);

  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}