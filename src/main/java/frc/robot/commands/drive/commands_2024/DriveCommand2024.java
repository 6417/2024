package frc.robot.commands.drive.commands_2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.MathUtilities;
import frc.fridowpi.utils.Vector2;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerve_2024.SwerveDrive2024;

public class DriveCommand2024 extends Command {

	public DriveCommand2024() {
		addRequirements(SwerveDrive2024.getInstance());
	}

	private boolean joystickNotInDeadBand() {
		boolean result = false;
		result |= Constants.SwerveDrive.Swerve2024.deadBand < Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getX());
		result |= Constants.SwerveDrive.Swerve2024.deadBand < Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getY());
		result |= Constants.SwerveDrive.Swerve2024.deadBand < Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getZ());
		return result;
	}

	private static class JoystickInput {
		public double x;
		public double y;
		public double r;
	}

	private Vector2 getXYvectorWithAppliedDeadBandFromJoystick() {
		var joystick = JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId);
		var x = joystick.getX();
		var y = joystick.getY();
		if (Math.abs(x) < Constants.SwerveDrive.Swerve2024.deadBand) {
			x = 0;
		} else {
			x = MathUtilities.map(Math.abs(x), Constants.SwerveDrive.Swerve2024.deadBand, 1.0, 0.0, 1.0)
					* Math.signum(x);
		}
		if (Math.abs(y) < Constants.SwerveDrive.Swerve2024.deadBand) {
			y = 0;
		} else {
			y = MathUtilities.map(Math.abs(y), Constants.SwerveDrive.Swerve2024.deadBand, 1.0, 0.0, 1.0)
					* Math.signum(y);
		}
		return new Vector2(x, y);
	}

	private JoystickInput applyDeadBandToXandY() {
		JoystickInput result = new JoystickInput();
		boolean xyNotInDeadBand = Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId)
						.getX()) > Constants.SwerveDrive.Swerve2024.deadBand
				|| Math.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId)
						.getY()) > Constants.SwerveDrive.Swerve2024.deadBand;
		if (xyNotInDeadBand) {
			Vector2 xyVector = getXYvectorWithAppliedDeadBandFromJoystick();
			result.x = xyVector.x;
			result.y = xyVector.y;
		} else {
			result.x = 0.0;
			result.y = 0.0;
		}
		return result;
	}

	private double getJoystickRotationWithAppliedDeadBand() {
		boolean rotaionNotInDeadBand = Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId)
						.getZ()) > Constants.SwerveDrive.Swerve2024.deadBand;
		if (rotaionNotInDeadBand) {
			return MathUtilities.map(
					JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getZ(),
					Constants.SwerveDrive.Swerve2024.deadBand, 1.0, 0.0, 1.0);
		}
		return 0.0;
	}

	private JoystickInput applyDeadBandToJoystickInput() {
		JoystickInput result = applyDeadBandToXandY();
		result.r = getJoystickRotationWithAppliedDeadBand();
		return result;
	}

	private Vector2 convertJoystickInputToVector(JoystickInput xyr) {
		double xSpeed = SwerveDrive2024.joystickInputToMetersPerSecond(xyr.x);
		double ySpeed = SwerveDrive2024.joystickInputToMetersPerSecond(xyr.y);
		if (Constants.SwerveDrive.Swerve2024.joystickYinverted)
			ySpeed *= -1;
		if (Constants.SwerveDrive.Swerve2024.joystickXinverted)
			xSpeed *= -1;
		return new Vector2(xSpeed, ySpeed);
	}

	private void setChassisSpeeds(Vector2 xyVector, double rotationSpeed) {
		switch (SwerveDrive2024.getInstance().getDriveMode()) {
			case Forwards:
				SwerveDrive2024.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
						rotationSpeed, new Rotation2d(0.0)));
				break;
			case Backwards:
				SwerveDrive2024.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
						rotationSpeed, new Rotation2d(Math.PI)));
				break;
			case FieldOriented:
				SwerveDrive2024.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
						rotationSpeed, Rotation2d.fromDegrees(FridoNavx.getInstance().getAngle())));
				break;
		}
	}

	@Override
	public void execute() {
		if (SwerveDrive2024.getInstance().areAllModulesZeroed() && joystickNotInDeadBand()) {
			JoystickInput xyr = applyDeadBandToJoystickInput();
			Vector2 xyVector = convertJoystickInputToVector(xyr);
			double rotationSpeed = xyr.r * Constants.SwerveDrive.Swerve2024.maxRotationSpeed;
			setChassisSpeeds(xyVector, rotationSpeed);
		} else {
			SwerveDrive2024.getInstance().stopAllMotors();
		}
	}
}
