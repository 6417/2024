package frc.robot.joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.WPIJoystick;
import frc.fridowpi.joystick.joysticks.POV;
import frc.robot.Constants;

public class XBoxOneController extends WPIJoystick {
    private XboxController xboxController;

	public XBoxOneController(IJoystickId port) {
		super(port);
		setThrottleChannel(5);
		setTwistChannel(4);
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
            return getLtValue() >= Constants.Joystick.lt_rt_reshold;
        }
        return getRtValue() >= Constants.Joystick.lt_rt_reshold;
    }
}
