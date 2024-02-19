package frc.fridowpi.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.joysticks.Xbox360Extended;
import frc.robot.Constants;

public class XBoxJoystick extends Joystick implements IJoystick {
    private XboxController xboxController;

    public XBoxJoystick(IJoystickId port) {
        super(port.getPort());
        super.setThrottleChannel(4);
        super.setTwistChannel(5);
        xboxController = new XboxController(port.getPort());
    }

    public double getLtValue() {
        return xboxController.getLeftTriggerAxis();
    }

    public double getRtValue() {
        return xboxController.getRightTriggerAxis();
    }

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id instanceof Xbox360Extended && id.getButtonId() >= 100 && id.getButtonId() < 200) {
            return new Trigger(() -> isPressedPOV((Xbox360Extended) id));
        } else if (id instanceof Xbox360Extended && id.getButtonId() >= 200 && id.getButtonId() < 300) {
            return new Trigger(() -> isPressedLorRT((Xbox360Extended) id));
        }
        return new JoystickButton(this, id.getButtonId());

    }

    private boolean isPressedPOV(Xbox360Extended id) {
        return id.pov.get().getDegrees() == getPOV();
    }

    private boolean isPressedLorRT(Xbox360Extended id) {
        if (id == Xbox360Extended.Lt) {
            return getLtValue() >= Constants.Joystick.treshold;
        }
        return getRtValue() >= Constants.Joystick.treshold;
    }
}
