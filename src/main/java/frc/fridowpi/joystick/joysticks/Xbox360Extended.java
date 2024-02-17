package frc.fridowpi.joystick.joysticks;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    Lt(200),
    Rt(201);

    private final int buttonId;
    public final Optional<Pov> pov;

    private Xbox360Extended(int id, Pov pov) {
        buttonId = id;
        this.pov = Optional.of(pov);
    }
    private Xbox360Extended(int id) {
        buttonId = id;
        pov = Optional.empty();
    }

    @Override
    public int getButtonId() {
        return buttonId;
    }
}
