package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.robot.Constants;

public class IdsWithState implements IJoystickButtonId {

	/*
	 * The state of the game:
	 *
	 *  ALL - always active					: Drive for example
	 *  DEFAULT - default state				: Shooter
	 *  ENDGAME - endgame state				: Climber
	 *
	 *  SYSID_TUNING - for sysid tuning only
	 */
    public enum State {
		ALL,
        DEFAULT,
        ENDGAME,
        SYSID_TUNING
    }

    public static State activeState = State.DEFAULT;
    public static int highestId = Constants.Joystick.idCounterStart;

    private int id;
    private IJoystickButtonId button;
	private State state = State.DEFAULT;

	public static IdsWithState create(IJoystickButtonId button, State state) {
		return new IdsWithState(highestId++, button, state);
	}

    private IdsWithState(int id, IJoystickButtonId button, State state) {
        this.id = id;
        this.button = button;
    }

    @Override
    public int getButtonId() {
        return id;
    }
    
    Trigger toButtonOnJoystick(IJoystick j) {
        return new Trigger(() -> isActivated(j));
    }

    private boolean isActivated(IJoystick j) {
        return j.getButton(button).getAsBoolean() && activeState == state || activeState == State.ALL;
    }

	public static IdsWithState from(IJoystickButtonId id2) {
		return new IdsWithState(highestId++, id2, State.ALL);
	}
}
