package frc.robot.joystick;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.joystick.joysticks.POV;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BDrive.SpeedFactor;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.ShooterSubsystem.ShooterConfig;

/**
 * JoystickBindings2024
 */
public class JoystickBindings2024 {
	private static JoystickBindings2024 instance = new JoystickBindings2024();
	private static List<Binding> tmp_bindings = List.of();

	public static JoystickBindings2024 getInstance() {
		return instance;
	}

	public static List<Binding> getBindingsLogitechTest() {
		tmp_bindings = List.of();
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
		return tmp_bindings;
	}

	public static List<Binding> getBindingsTankdriveLogitech() {
		tmp_bindings = List.of();
		quickBindWhileHeld(Logitech.lt, () -> Config.drive().setSpeedFactor(
				Config.data().drive().speedFactors().get(SpeedFactor.FAST)));
		quickBindWhileHeld(Logitech.rt, () -> Config.drive().setSpeedFactor(
				Config.data().drive().speedFactors().get(SpeedFactor.SLOW)));

		// TODO: make better CONFIG
		// Config.active.getShooter().ifPresent(s -> {
		// 	quickBind(Logitech.a, State.DEFAULT, () -> s.shoot(ShooterConfig.INTAKE));
		// 	quickBind(Logitech.b, State.DEFAULT, () -> s.shoot(ShooterConfig.AMP));
		// 	quickBind(Logitech.y, State.DEFAULT, () -> s.shoot(ShooterConfig.SPEAKER));
		// 	quickBind(Logitech.x, State.DEFAULT, () -> s.setSpeedPercent(0));
		// });

		// Config.active.getClimber().ifPresent(climber -> {
		// 	quickBind(Logitech.y, State.ENDGAME, climber::oneStepUp);
		// 	quickBind(Logitech.a, State.ENDGAME, climber::oneStepDown);
		// 	quickBind(Logitech.x, State.ENDGAME, climber::release);
		// 	quickBind(Logitech.b, State.ENDGAME, climber::retract);
		// });
		//
		quickBind(Logitech.b, () -> Config.active.getClimber().get().getServo().setSpeed(-0.1));
		quickBind(Logitech.a, () -> Config.active.getClimber().get().getServo().setSpeed(0.1));

		return tmp_bindings;
	}

	// On single press
	public static void quickBind(IJoystickButtonId button, State state, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, IdsWithState.from(button),
				Trigger::onTrue, new InstantCommand(fn)));
	}

	public static void quickBind(IJoystickButtonId button, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button,
				Trigger::onTrue, new InstantCommand(fn)));
	}

	public static void quickBind(IJoystickButtonId button, Command cmd) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::onTrue, cmd));
	}

	// While held
	public static void quickBindWhileHeld(IJoystickButtonId button, State state, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, IdsWithState.from(button),
				Trigger::whileTrue, new InstantCommand(fn)));
	}

	public static void quickBindWhileHeld(IJoystickButtonId button, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button,
				Trigger::whileTrue, new InstantCommand(fn)));
	}

	public static void quickBindWhileHeld(IJoystickButtonId button, Command cmd) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::whileTrue, cmd));
	}

	// If using different States
	public static void setState(State state) {
		IdsWithState.activeState = state;
	}

}
