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
import frc.fridowpi.joystick.XBoxJoystick;
import frc.fridowpi.joystick.joysticks.XboxOne;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.drive.getAutonomousTrajectory;

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
		JoystickHandler.getInstance().setJoystickFactory(XBoxJoystick::new);
		JoystickHandler.getInstance().init(); // Don't ask, it works ;)
		JoystickHandler.getInstance().setupJoysticks(List.of(
				Constants.Joystick.primaryJoystickId));
		JoystickHandler.getInstance().init();

		// Set active state //
		IdsWithState.activeState = state;

		// Create bindings //
		JoystickHandler.getInstance().bindAll(JoystickBindings2024.getBindingsLogitechTest());
		JoystickHandler.getInstance().init();
	}

	public void run() {

		var x = getPrimaryJoystick().getX();
		var y = getPrimaryJoystick().getY();
		var tw = getPrimaryJoystick().getTwist();
		var thr = getPrimaryJoystick().getThrottle();
		var lt = XBoxJoystick.getLtValue();
		var rt = XBoxJoystick.getRtValue();
		
		if (x > 0.05) {
			System.out.println("X:" + x);
		}
		if (y > 0.05) {
			System.out.println("Y:" + y);
		}
		if (tw > 0.05) {
			System.out.println("Twist:" + tw);
		}
		if (thr > 0.05) {
			System.out.println("Throttle:" + thr);
		}
		if (lt > 0.05) {
			System.out.println("Left Trigger:" + lt);
		}
		if (rt > 0.05) {
			System.out.println("Right Trigger:" + rt);
		}
		

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

