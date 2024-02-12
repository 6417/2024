package frc.fridowpi.joystick.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridolib.Directions.JoystickLeft;
import frc.fridolib.Directions.Pov;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.robot.joystick.ControllerWithState;

public enum Xbox360Extended implements IJoystickButtonId {
    DPadUp(100, Pov.UP),
    DPadUpRight(101, Pov.UP_RIGHT),
    DPadRight(102, Pov.RIGHT),
    DPadDownRight(103, Pov.DOWN_RIGHT),
    DPadDown(104, Pov.DOWN),
    DPadDownLeft(105, Pov.DOWN_LEFT),
    DPadLeft(106, Pov.LEFT),
    DPadUpLeft(107, Pov.UP_LEFT),
    JoystickLeft(201, JoystickLeft.RIGHT);

    private final int buttonId;
    public final Pov pov;
    public final JoystickLeft joystickLeft;

    private Xbox360Extended(int id, Pov pov) {
        buttonId = id;
        this.pov = pov;
        this.joystickLeft
    }
    
    @Override
    public int getButtonId() {
        return buttonId;
    }
}
