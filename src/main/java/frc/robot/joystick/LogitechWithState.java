
package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.WPIJoystick;
import frc.robot.Constants;

/**
 * LogitechWithState
 */
public class LogitechWithState extends WPIJoystick {

	public LogitechWithState(IJoystickId port) {
		super(port);
	}

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id.getButtonId() >= Constants.Joystick.idCounterStart) {
            var idNew = (IdsWithState) id;
            return idNew.toButtonOnJoystick(this);
        }
        return super.getButton(id);
    }
}
