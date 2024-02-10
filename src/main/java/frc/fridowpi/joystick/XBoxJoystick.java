package frc.fridowpi.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.joysticks.Xbox360Extended;

public class XBoxJoystick extends Joystick implements IJoystick {
    public XBoxJoystick(IJoystickId port) {
        super(port.getPort());
    }

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id instanceof Xbox360Extended && id.getButtonId() >= 100) {
            return new Trigger(() -> ((Xbox360Extended) id).pov.getDegrees() == getDirectionDegrees());
        }
        return new JoystickButton(this, id.getButtonId());
    }

}
