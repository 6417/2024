package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.MathUtilities;
import frc.fridowpi.utils.Vector2;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerve.SwerveDrive;

public class DefaultDriveCommand extends Command {
    
    public DefaultDriveCommand() {
        addRequirements(SwerveDrive.getInstance());
    }

    private boolean joystickNotInDeadBand() {
        boolean result = false;
        result |= Constants.SwerveDrive.deadBand < Math.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getX());
        result |= Constants.SwerveDrive.deadBand < Math.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getY());
        result |= Constants.SwerveDrive.deadBand < Math.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getZ());
        return result;
    }

    private static class JoystickInput {
        public double x;
        public double y;
        public double r;
    }

    private Vector2 getXYvectorWithAppliedDeadBandFromJoystick() {
		var joystick = JoystickHandler.getInstance().getJoystick(Constants.Joystick.id);
		var x = joystick.getX();
		var y = joystick.getY();
		if (Math.abs(x) < Constants.SwerveDrive.deadBand) {
			x = 0;
		} else {
			x = MathUtilities.map(Math.abs(x), Constants.SwerveDrive.deadBand, 1.0, 0.0, 1.0) * Math.signum(x);
		}
		if (Math.abs(y) < Constants.SwerveDrive.deadBand) {
			y = 0;
		} else {
			y = MathUtilities.map(Math.abs(y), Constants.SwerveDrive.deadBand, 1.0, 0.0, 1.0) * Math.signum(y);
		}
		return new Vector2(x, y);
    }

    private JoystickInput applyDeadBandToXandY() {
        JoystickInput result = new JoystickInput();
        boolean xyNotInDeadBand = Math.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getX()) > Constants.SwerveDrive.deadBand
                || Math.abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getY()) > Constants.SwerveDrive.deadBand;
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
                .abs(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getZ()) > Constants.SwerveDrive.deadBand;
        if (rotaionNotInDeadBand)
            return MathUtilities.map(JoystickHandler.getInstance().getJoystick(Constants.Joystick.id).getZ(),
                    Constants.SwerveDrive.deadBand, 1.0, 0.0, 1.0);
        return 0.0;
    }

    private JoystickInput applyDeadBandToJoystickInput() {
        JoystickInput result = applyDeadBandToXandY();
        result.r = getJoystickRotationWithAppliedDeadBand();
        return result;
    }

    private Vector2 convertJoystickInputToVector(JoystickInput xyr) {
        double xSpeed = SwerveDrive.joystickInputToMetersPerSecond(xyr.x);
        double ySpeed = SwerveDrive.joystickInputToMetersPerSecond(xyr.y);
        if (Constants.SwerveDrive.joystickYinverted)
            ySpeed *= -1;
        if (Constants.SwerveDrive.joystickXinverted)
            xSpeed *= -1;
        return new Vector2(xSpeed, ySpeed);
    }

    private void setChassisSpeeds(Vector2 xyVector, double rotationSpeed) {
            switch (SwerveDrive.getInstance().getDriveMode()) {
                case ShooterBack:
                    SwerveDrive.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
                            rotationSpeed, new Rotation2d(0.0)));
                    break;
                case ShooterFront:
                    SwerveDrive.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
                            rotationSpeed, new Rotation2d(Math.PI)));
                    break;
                case FieldOriented:
                    SwerveDrive.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(xyVector.x, xyVector.y,
                            rotationSpeed, Rotation2d.fromDegrees(FridoNavx.getInstance().getAngle())));
                    break;
            }
    }

    @Override
    public void execute() {
        if (SwerveDrive.getInstance().areAllModulesZeroed() && joystickNotInDeadBand()) {
            JoystickInput xyr = applyDeadBandToJoystickInput();
            Vector2 xyVector = convertJoystickInputToVector(xyr);
            double rotationSpeed = xyr.r * Constants.SwerveDrive.maxRotationSpeed;
            setChassisSpeeds(xyVector, rotationSpeed);
        } else
            SwerveDrive.getInstance().stopAllMotors();
    }
}
