package frc.robot.commands.drive.commands_2024;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.MathUtilities;
import frc.fridowpi.utils.Vector2;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerve_2024.SwerveDrive2024;

public class OldDriveCommand2024 extends Command {
	private final SwerveDrive2024 drive;

	public OldDriveCommand2024() {
		assert Config.drive() instanceof SwerveDrive2024 : "Not implemented for " + Config.drive().getClass();
		drive = (SwerveDrive2024) Config.drive();
		addRequirements(Config.drive());
	}

	private boolean joystickNotInDeadBand() {
		boolean result = false;
		result |= Constants.SwerveDrive.Swerve2024.deadBand < Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getX());
		result |= Constants.SwerveDrive.Swerve2024.deadBand < Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getY());
		result |= Constants.SwerveDrive.Swerve2024.deadBand < Math
				.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getTwist());
		return result;
	}

	private static class JoystickInput {
		public double x;
		public double y;
		public double r;
	}

	private Vector2 getXYvectorWithAppliedDeadBandFromJoystick() {
		var joystick = JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId);
		var y = joystick.getY();
		var x = -joystick.getX();
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
						.getTwist()) > Constants.SwerveDrive.Swerve2024.deadBand;
		if (rotaionNotInDeadBand) {
			return MathUtilities.map(
					JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getTwist(),
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
		double xSpeed = Config.drive().percent2driveVelocity(xyr.x).in(MetersPerSecond);
		double ySpeed = Config.drive().percent2driveVelocity(xyr.y).in(MetersPerSecond);
		if (Constants.SwerveDrive.Swerve2024.joystickYinverted)
			ySpeed *= -1;
		if (Constants.SwerveDrive.Swerve2024.joystickXinverted)
			xSpeed *= -1;
		return new Vector2(xSpeed, ySpeed);
	}

	private void setChassisSpeeds(Vector2 xyVector, double rotationSpeed) {
		switch (Config.drive().getOrientation()) {
			case Forwards:
				Config.drive().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
						rotationSpeed, new Rotation2d(0.0)));
				break;
			case Backwards:
				Config.drive().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
						rotationSpeed, new Rotation2d(Math.PI)));
				break;
			case FieldOriented:
				Config.drive().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
						rotationSpeed, Rotation2d.fromDegrees(FridoNavx.getInstance().getAngle())));
				break;
		}
	}

	private void applySkewLimit(Vector2 vec) {
		/*
		 * vec.x *= vec.x * Math.signum(vec.x);
		 * vec.y *= vec.y * Math.signum(vec.y);
		 */
	}

	@Override
	public void execute() {
		if (drive.areAllModulesZeroed() && joystickNotInDeadBand()) {
			JoystickInput xyr = new JoystickInput();
			xyr.x = JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getX();
			xyr.y = JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getY();
			xyr.r = -JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getTwist();

			Vector2 xyVector = convertJoystickInputToVector(xyr);
			if (xyVector.magnitude() < Constants.SwerveDrive.Swerve2024.deadBand) {
				xyVector.x = 0;
				xyVector.y = 0;
			} else {
				double mag = xyVector.magnitude();
				double normalizedMag = MathUtilities.map(mag, Constants.SwerveDrive.Swerve2024.deadBand, 1.0, 0.0, 1.0);
				xyVector.normalize();
				xyVector = xyVector.scaled(normalizedMag);
			}
			// applySkewLimit(xyVector);
			if (Math.abs(xyr.r) < Constants.SwerveDrive.Swerve2024.deadBand) {
				xyr.r = 0;
			} else {
				xyr.r = MathUtilities.map(Math.abs(xyr.r), Constants.SwerveDrive.Swerve2024.deadBand, 1, 0, 1)
						* Math.signum(xyr.r);
			}
			double rotationSpeed = xyr.r * Constants.SwerveDrive.Swerve2024.maxRotationSpeed;
			setChassisSpeeds(xyVector, rotationSpeed);
		} else {
			Config.drive().stopAllMotors();
		}
	}
}
