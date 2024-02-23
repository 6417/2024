package frc.robot.joystick;

import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.JoystickHandler;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.joystick.IdsWithState.State;

// Singleton that manages the joystick configuration of 2024 //
public class Joystick2024 implements Sendable {
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
		// JoystickHandler.getInstance().setJoystickFactory(ControllerWithState::new);
		JoystickHandler.getInstance().init(); // Don't ask, it works ;)
		JoystickHandler.getInstance().setupJoysticks(List.of(
				Constants.Joystick.primaryJoystickId));
		JoystickHandler.getInstance().init();

		// Set active state //
		IdsWithState.activeState = state;

		// Create bindings //
		JoystickHandler.getInstance().bind(Config.drive());
		// Config.active.getShooter().ifPresent(s ->
		// JoystickHandler.getInstance().bind(s));
	}

	public void run() {
		if (JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId) != null) {
			var js = getPrimaryJoystick();
			if (js.getX() > 0.1) {
				System.out.println("X: " + js.getX());
			}
			if (js.getY() > 0.1) {
				System.out.println("Y: " + js.getY());
			}
			if (js.getThrottle() > 0.1) {
				System.out.println("Throttle: " + js.getThrottle());
			}
			if (js.getTwist() > 0.1) {
				System.out.println("Twist: " + js.getTwist());
			}
		}
	}

	public void quickBind(IJoystickButtonId button, Runnable fn) {
		JoystickHandler.getInstance()
				.bind(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::onTrue,
						new InstantCommand(fn)));
	}

	public void quickBind(IJoystickButtonId button, Command cmd) {
		JoystickHandler.getInstance()
				.bind(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::onTrue, cmd));
	}

	public void quickBindWhileHeld(IJoystickButtonId button, Command cmd) {
		JoystickHandler.getInstance()
				.bind(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::whileTrue, cmd));
	}

	// Setters //
	public void setState(State state) {
		IdsWithState.activeState = state;
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

