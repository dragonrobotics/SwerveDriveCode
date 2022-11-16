package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double currentDegrees = currentAngle.getDegrees(), desiredDegrees = desiredState.angle.getDegrees(), speed = desiredState.speedMetersPerSecond;
    double TDM90, TDM180, TDM360, targetSpeed, targetAngle; // tdm being target delta mod
    double targetDelta = desiredDegrees - currentDegrees;
    TDM90 = targetDelta%90;
    TDM180 = targetDelta%180;
    TDM360 = targetDelta%360;
    if(TDM90 == TDM180 && TDM90 == TDM360) {
      targetSpeed = speed;
      targetAngle = currentDegrees + TDM90;
    } else if(TDM90 == TDM180) {
      targetSpeed = 0 - speed;
      targetAngle = currentDegrees + TDM90;
    } else if(TDM180 == TDM360) {
      targetSpeed = 0 - speed;
      targetAngle = currentDegrees - (90 - TDM90);
    } else {
      targetSpeed = speed;
      targetAngle = currentDegrees - (90 - TDM90);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
}
