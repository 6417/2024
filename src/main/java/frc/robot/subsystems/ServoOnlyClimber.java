package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridolib.QuickCmd;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.motors.FridoServoMotor;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BClimber;

public class ServoOnlyClimber extends BClimber {
	private FridoServoMotor servoLinks;
	private FridoServoMotor servoRechts;

	/** Creates a new ClimberSubsystem. */
	public ServoOnlyClimber() {
	}

	@Override
	public void init() {
		servoLinks = new FridoServoMotor(Constants.Climber.servoLinksId);
		servoRechts = new FridoServoMotor(Constants.Climber.servoRechtsId);

		servoRechts.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
		servoRechts.setMaxAngle(130);
		servoLinks.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
		servoLinks.setMaxAngle(130);

		servoLinks.setAngle(Constants.Climber.servoLeftLockAngle);
		servoRechts.setAngle(Constants.Climber.servoRightLockAngle);
	}

	@Override
	public void run() {
	}

	@Override
	public void release() {
		System.out.println("test erfolgreich");
		new SequentialCommandGroup(
				QuickCmd.withInit(this::releaseServos),
				new WaitCommand(1),
				QuickCmd.withInit(this::lockServos)).schedule();
	}

	public void releaseServos() {
		servoRechts.setAngle(Constants.Climber.ersatzServoRightReleaseAngle);
		servoLinks.setAngle(Constants.Climber.ersatzServoLeftReleaseAngle);
	}

	public void lockServos() {
		servoRechts.setAngle(Constants.Climber.ersatzServoRightLockAngle);
		servoLinks.setAngle(Constants.Climber.ersatzServoLeftLockAngle);
	}

	@Override
	public ClimberData getData() {
		var motorLeft = 22;
		var motorRight = 21;
		var servo = -1;
		return new ClimberData(List.of(motorLeft, motorRight, servo));
	}

	public FridoServoMotor getServoLeft() {
		return servoLinks;
	}

	public FridoServoMotor getServoRight() {
		return servoRechts;
	}

	// nothing

	@Override
	public void retract() {
	}

	@Override
	public void oneStepUp(double speed) {
		servoLinks.setAngle(servoLinks.getAngle() + speed);
		servoRechts.setAngle(servoRechts.getAngle() + speed);
		System.out.println("servo Links: " + servoLinks.getAngle() + "   Servo Rechts: " + servoRechts.getAngle());
	}

	@Override
	public void stop() {
	}
}