package frc.fridowpi.joystick.joysticks;

import frc.fridowpi.joystick.IJoystickButtonId;

public enum Xbox360 implements IJoystickButtonId {
    x(1),
    a(2),
    b(3),
    y(4),
    lb(5),
    rb(6),
    lt(7),
    rt(8),
    back(9),
    start(10),
    leftJoystick(11),
    rightJoystick(12);

    private final int buttonId;

    private Xbox360(int id) {
        buttonId = id;
    }

    @Override
    public int getButtonId() {
        return buttonId;
    }
}
