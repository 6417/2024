package frc.robot.commands.drive.commands_2024;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.Vector2;
import frc.robot.Config;
import frc.robot.Controls;
import frc.robot.joystick.Joystick2024;

/**
 * DriveCommand
 */
public class DriveCommand2024 extends FridoCommand {

	private SlewRateLimiter xLimiter = new SlewRateLimiter(Controls.getSlewRateLimit(), -1000, 0);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(Controls.getSlewRateLimit(), -1000, 0);
	private SlewRateLimiter rotLimiter = new SlewRateLimiter(Controls.getSlewRateLimit(), -1000, 0);

	private double maxSpeed = 0;

	public DriveCommand2024() {
		addRequirements(Config.drive());
	}

	@Override
	public void execute() {
		var speed = abs(Config.drive().getSwerveWheelSpeeds().in(MetersPerSecond));
		if (speed > maxSpeed) {
			maxSpeed = speed;
			System.out.println("MaxSpeed: " + speed);
		}

		var joystick = Joystick2024.getInstance().getPrimaryJoystick();
		var xy = new Vector2(joystick.getX(), joystick.getY());
		var rot = -joystick.getTwist();

		if (Controls.getControlMode() == Controls.ControlMode.SEPARATE_ACCELERATION) {
			xy = xy.normalized().scaled(joystick.getZ());
			System.out.println("Z: " + joystick.getZ());
		}

		// Brake if input is 0
		if (xy.magnitude() < Controls.getDeadBandDrive()
				&& abs(rot) < Controls.getDeadBandTurn()) {
			Config.drive().stopAllMotors();
			return;
		}

		// Apply deadband
		xy = applyDeadband(xy, Controls.getDeadBandDrive())
				.scaled(Controls.getAccelerationSensitivity());
		rot = applyDeadband(rot, Controls.getDeadBandTurn())
				* Controls.getTurnSensitivity();

		// Apply slew rate
		if (Controls.isSlewRateLimited()) {
			var xLimited = xLimiter.calculate(abs(xy.x)) * signum(xy.x);
			var yLimited = yLimiter.calculate(abs(xy.y)) * signum(xy.y);
			// rot = rotLimiter.calculate(abs(rot)) * signum(rot);
			if (Double.isNaN(xLimited) || Double.isNaN(yLimited)) {
				xLimiter.reset(xy.x);
				yLimiter.reset(xy.y);
			} else {
				xy = new Vector2(xLimited, yLimited);
			}
		}

		// Convert to velocity
		xy.x = Config.drive().percent2driveVelocity(xy.x).in(MetersPerSecond);
		xy.y = Config.drive().percent2driveVelocity(xy.y).in(MetersPerSecond);
		rot = Config.drive().percent2rotationVelocityDouble(rot);

		setChassisSpeeds(xy, rot);
		System.out.println(xy.toString() + rot);
	}

	public static Vector2 applyDeadband(Vector2 xy, double deadBand) {
		return xy.normalized().scaled(applyDeadband(xy.magnitude(), deadBand));
	}

	public static double applyDeadband(double x, double deadBand) {
		return abs(x) < deadBand ? 0 : (abs(x) - deadBand) / (1 - deadBand) * signum(x);
	}

	private void setChassisSpeeds(Vector2 vxy, double vRot) {
		switch (Config.drive().getOrientation()) {
			case Forwards:
				Config.drive().drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
						vRot, new Rotation2d(0.0)));
				break;
			case Backwards:
				Config.drive().drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
						vRot, Rotation2d.fromRadians(Math.PI)));
				break;
			case FieldOriented:
				Config.drive().drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
						vRot, Rotation2d.fromDegrees(FridoNavx.getInstance().getAngle())));
				break;
		}
	}
}
