package frc.robot.commands.drive.commands_2024;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

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

	public DriveCommand2024() {
		addRequirements(Config.drive());
	}

	@Override
	public void execute() {
		var joystick = Joystick2024.getInstance().getPrimaryJoystick();
		var xy = new Vector2(joystick.getX(), joystick.getY());
		var rot = -joystick.getTwist();

		if (xy.magnitude() < Controls.getDriveDeadband()
				&& abs(rot) < Controls.getTurnDeadband()) {
			Config.drive().stopAllMotors();
			return;
		}

		xy = applyDeadband(xy, Controls.getDriveDeadband())
				.scaled(Controls.getAccelerationSensitivity());
		rot = applyDeadband(rot, Controls.getTurnDeadband())
				* Controls.getTurnSensitivity();

		xy.x = Config.drive().percent2driveVelocity(xy.x).in(MetersPerSecond);
		xy.y = Config.drive().percent2driveVelocity(xy.y).in(MetersPerSecond);
		rot = Config.drive().percent2rotationVelocityDouble(rot);

		setChassisSpeeds(xy, rot);
	}

	private Vector2 applyDeadband(Vector2 xy, double deadBand) {
		return xy.normalized().scaled(applyDeadband(xy.magnitude(), deadBand));
	}

	private double applyDeadband(double x, double deadBand) {
		return abs(x) < Controls.getDriveDeadband() ? 0 : (abs(x) - deadBand) / (1 - deadBand) * signum(x);
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
