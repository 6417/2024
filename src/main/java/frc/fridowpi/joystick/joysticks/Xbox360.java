package frc.fridowpi.joystick.joysticks;

import frc.fridowpi.joystick.IJoystickButtonId;

public enum Xbox360 implements IJoystickButtonId {
    a(1),
    b(2),
    x(3),
    y(4),
    lb(5),
    rb(6),
    lt(8),
    rt(9),
    back(10),
    start(11),
    leftJoystick(12),
    rightJoystick(13);

    private final int buttonId;

    private Xbox360(int id) {
        buttonId = id;
    }
    @Override
    public int getButtonId() {
        return buttonId;
    }
}
