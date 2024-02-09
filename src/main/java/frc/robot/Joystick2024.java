package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.JoystickHandler;
import frc.robot.commands.climber.ClimberManuel;

public class Joystick2024 {
    private static Joystick2024 instance = new Joystick2024();
    final public static Joystick joystick = new Xbox360(0);

    private Joystick2024() {
        JoystickHandler.getInstance().getJoystick());
        JoystickHandler.getInstance().bindAll(
            new Binding(, 0,  (a, b) -> {}, new ClimberManuel(true)),
            new Binding(null, null, null, null)
        );
    }

    public static Joystick2024 getInstance() {
        return instance;
    }
}
