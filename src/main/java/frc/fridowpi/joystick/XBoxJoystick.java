package frc.fridowpi.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.fridowpi.joystick.joysticks.POV;

public class XBoxJoystick extends Joystick implements IJoystick {
    private static XboxController xboxController;

    public XBoxJoystick(IJoystickId port) {
        super(port.getPort());
        super.setTwistChannel(4);
        super.setThrottleChannel(5);
        super.setXChannel(0);
        super.setYChannel(1);
        xboxController = new XboxController(port.getPort());
    }

    public static double getLtValue() {
        return xboxController.getLeftTriggerAxis();
    }

    public static double getRtValue() {
        return xboxController.getRightTriggerAxis();
    }

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id instanceof POV && id.getButtonId() >= 100 && id.getButtonId() < 200) {
            return new Trigger(() -> isPressedPOV((POV) id));
        } else if (id instanceof POV && id.getButtonId() >= 200 && id.getButtonId() < 300) {
            return new Trigger(() -> isPressedLorRT((POV) id));
        }
        return new JoystickButton(this, id.getButtonId());

    }

    private boolean isPressedPOV(POV id) {
        return id.pov.get().getDegrees() == getPOV();
    }

    private boolean isPressedLorRT(POV id) {
        if (id == POV.Lt) {
            return getLtValue() >= Constants.Joystick.treshold;
        }
        return getRtValue() >= Constants.Joystick.treshold;
    }
}
