package frc.fridowpi.joystick.joysticks;

import frc.fridowpi.joystick.IJoystickButtonId;

public enum XboxOne implements IJoystickButtonId{
    a(1),
    b(2),
    x(3),
    y(4),
    lb(5),
    rb(6),
    lt(12),
    rt(11),
    back(7),
    start(8),
    leftJoystick(9),
    rightJoystick(10);

    private final int buttonId;

    private XboxOne (int buttonId){
        this.buttonId = buttonId;
    }
    @Override
    public int getButtonId() {
        return buttonId;
    }
    
}
