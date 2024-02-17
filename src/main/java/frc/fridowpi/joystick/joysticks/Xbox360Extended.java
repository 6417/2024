package frc.fridowpi.joystick.joysticks;

import frc.fridolib.Directions.Pov;
import frc.fridowpi.joystick.IJoystickButtonId;

public enum Xbox360Extended implements IJoystickButtonId {
    DPadUp(100, Pov.UP),
    DPadUpRight(101, Pov.UP_RIGHT),
    DPadRight(102, Pov.RIGHT),
    DPadDownRight(103, Pov.DOWN_RIGHT),
    DPadDown(104, Pov.DOWN),
    DPadDownLeft(105, Pov.DOWN_LEFT),
    DPadLeft(106, Pov.LEFT),
    DPadUpLeft(107, Pov.UP_LEFT);

    private final int buttonId;
    public final Pov pov;

    private Xbox360Extended(int id, Pov pov) {
        buttonId = id;
        this.pov = pov;
    }
    
    @Override
    public int getButtonId() {
        return buttonId;
    }
}
