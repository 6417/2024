package frc.robot.joystick;

import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.JoystickHandler;
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
		// JoystickHandler.getInstance().setJoystickFactory(XBoxOneController::new);
		JoystickHandler.getInstance().init();
		JoystickHandler.getInstance().setupJoysticks(List.of(
				Constants.Joystick.secondaryJoystickId));

		JoystickHandler.getInstance().setJoystickFactory(XBoxOneController::new);
		JoystickHandler.getInstance().init(); // Don't ask, it works ;)
		JoystickHandler.getInstance().setupJoysticks(List.of(
				Constants.Joystick.primaryJoystickId));
		JoystickHandler.getInstance().init();

		// Set active state //
		IdsWithState.activeState = state;

		// Create bindings //
		JoystickHandler.getInstance().bindAll(JoystickBindings2024.getBindingsSwerve2024());
		JoystickHandler.getInstance().bindAll(JoystickBindings2024.getBindingsLogitechTest());
		JoystickHandler.getInstance().init();
	}

	public void run() {
		// if (JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId) != null) {
			// var js = getPrimaryJoystick();
		// 	if (IdsWithState.activeState == State.ENDGAME && js.getY() > 0.1) {
		// 		Config.active.getClimber().ifPresent((climber) -> climber.oneStepUp(js.getY()));
		// 	}
		// }
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

