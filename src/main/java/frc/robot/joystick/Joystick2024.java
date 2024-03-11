package frc.robot.joystick;

import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.XBoxJoystick;
import frc.fridowpi.module.Module;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.drive.getAutonomousTrajectory;

// Singleton that manages the joystick configuration of 2024 //
public class Joystick2024 extends Module {
	private static Joystick2024 instance = new Joystick2024();

	public IJoystick getPrimaryJoystick() {
		return JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId);
	}

	private Joystick2024() {
	}

	public static Joystick2024 getInstance() {
		return instance;
	}

	public void setup(State state) {
		JoystickHandler.getInstance().setJoystickFactory(LogitechWithState::new);
		JoystickHandler.getInstance().init(); // Don't ask, it works ;)
		JoystickHandler.getInstance().setupJoysticks(List.of(
				Constants.Joystick.primaryJoystickId));
		JoystickHandler.getInstance().init();

		// Set active state //
		IdsWithState.activeState = state;

		// Create bindings /
		JoystickHandler.getInstance().bindAll(JoystickBindings2024.getBindingsSwerve2024());
		JoystickHandler.getInstance().init();
	}

	@Override
	public void periodic() {
		var joystickY = getPrimaryJoystick().getY();
		var joystickX = getPrimaryJoystick().getX();
		var joystickTwist = getPrimaryJoystick().getTwist();
		var joystickTrottel = getPrimaryJoystick().getThrottle();

		if (joystickY >= 0.05) {
			System.out.println("Y " + joystickY);
		}
		if (joystickX >= 0.05) {
			System.out.println("X " + joystickX);
		}
		if (joystickTwist >= 0.05) {
			System.out.println("Twist " + joystickTwist);
		}
		if (joystickTrottel >= 0.05) {
			System.out.println("Trottel " + joystickTrottel);
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
	}
}
// getX() -> joystick left x
// getY() -> joystick left y
// getThrottle() -> joystick right x
// getTwist() -> joystick right y
// getMagnitude() -> how close the joystick is to joystick centre
// getDirection() -> direction from -180 to 180 Degrees, joystick left
