package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.joystick.joysticks.POV;
import frc.robot.Constants;
import frc.robot.joystick.IdsWithState.State;

/**
 * JoystickBindings2024
 */
public class JoystickBindings2024 {
	private static JoystickBindings2024 instance = new JoystickBindings2024();

	public static JoystickBindings2024 getInstance() {
		return instance;
	}

	public void bindAllLogitech() {
		quickBindWhileHeld(Logitech.a, () -> System.out.println("a"));
		quickBind(Logitech.b, () -> System.out.println("b"));
		quickBind(Logitech.x, () -> System.out.println("x"));
		quickBind(Logitech.y, () -> System.out.println("y"));
		quickBind(Logitech.start, () -> System.out.println("start"));
		quickBind(Logitech.back, () -> System.out.println("back"));
		quickBind(Logitech.lb, () -> System.out.println("lb"));
		quickBind(Logitech.rb, () -> System.out.println("rb"));
		quickBind(Logitech.lt, () -> System.out.println("lt"));
		quickBind(Logitech.rt, () -> System.out.println("rt"));

		quickBind(POV.DPadUp, () -> System.out.println("dpad up"));
		quickBind(POV.DPadUpRight, () -> System.out.println("dpad up right"));
		quickBind(POV.DPadRight, () -> System.out.println("dpad right"));
		quickBind(POV.DPadDownRight, () -> System.out.println("dpad down right"));
		quickBind(POV.DPadDown, () -> System.out.println("dpad down"));
		quickBind(POV.DPadDownLeft, () -> System.out.println("dpad down left"));
		quickBind(POV.DPadLeft, () -> System.out.println("dpad left"));
		quickBind(POV.DPadUpLeft, () -> System.out.println("dpad up left"));

		quickBind(POV.Lt, () -> System.out.println("POV lt"));
		quickBind(POV.Rt, () -> System.out.println("POV rt"));
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

	public void quickBindWhileHeld(IJoystickButtonId button, Runnable fn) {
		JoystickHandler.getInstance()
				.bind(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::whileTrue,
						new InstantCommand(fn)));
	}

	public void quickBindWhileHeld(IJoystickButtonId button, Command cmd) {
		JoystickHandler.getInstance()
				.bind(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::whileTrue, cmd));
	}

	// Setters //
	public void setState(State state) {
		IdsWithState.activeState = state;
	}

}
