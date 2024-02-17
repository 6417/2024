package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.joysticks.Xbox360;

public enum IdsWithState implements IJoystickButtonId {
    A(1000, Xbox360.a),
    B(1001, Xbox360.b),
    X(1002, Xbox360.x),
    Y(1003, Xbox360.y),
    LB(1004, Xbox360.lb),
    RB(1005, Xbox360.rb),
    BACK(1006, Xbox360.back),
    START(1007, Xbox360.start);

    public enum State{
        DEFAULT,
        ENDGAME,
        SYSID_TUNING
    }

    private int id;
    private IJoystickButtonId button;
    public static State state = State.DEFAULT;

    private IdsWithState(int id, IJoystickButtonId button) {
        this.id = id;
        this.button = button;
    }

    @Override
    public int getButtonId() {
        return id;
    }
    
    Trigger toButtonOnJoystick(ControllerWithState j) {
        return new Trigger(() -> isActivated(j));
    }

    private boolean isActivated(ControllerWithState j) {
        return j.getButton(button).getAsBoolean() && state == State.DEFAULT;
    }
}
