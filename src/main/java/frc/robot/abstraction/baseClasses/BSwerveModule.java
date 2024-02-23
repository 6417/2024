
package frc.robot.abstraction.baseClasses;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config;

/**
 * BSwerveModule
 */
public abstract class BSwerveModule implements Sendable {

	abstract public void driveForward(double speed);

	abstract public void rotate(double speed);

	abstract public void setEncoderZeroedFalse();

	abstract public void setRotationEncoderTicks(double ticks);

	abstract public void stopAllMotors();

	abstract public double getRotationEncoderTicks();

	abstract public void setDesiredState(SwerveModuleState state);

	abstract public void setDesiredRotationMotorTicks(double position);

	abstract public void setCurrentRotationToEncoderHome();

	abstract public void setIdleMode(IdleMode mode);

	abstract public boolean hasEncoderBeenZeroed();

	abstract public Config getConfig();

	abstract public void zeroEncoder();
}
