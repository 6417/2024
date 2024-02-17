package frc.robot.joystick;

import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.XBoxJoystick;
import frc.fridowpi.joystick.joysticks.Xbox360;
import frc.fridowpi.joystick.joysticks.Xbox360Extended;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.commands.SimplePrintCommand;
import frc.robot.commands.climber.ClimberManuel;
import frc.robot.commands.climber.ClimberRelease;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Joystick2024 implements Sendable {
    private static Joystick2024 instance = new Joystick2024();

    IJoystickButtonId btnId = Xbox360.a;

    private Joystick2024() {
        JoystickHandler.getInstance().bindAll(List.of(
                new Binding(() -> 0, Xbox360.y, Trigger::toggleOnTrue, new ClimberManuel(true)),
                new Binding(() -> 0, Xbox360.a, Trigger::toggleOnTrue, new ClimberManuel(false))));
        JoystickHandler.getInstance().getJoystick(() -> 0);
    }

    public void setup(State state) {
        JoystickHandler.getInstance().setJoystickFactory(XBoxJoystick::new);
        JoystickHandler.getInstance().setupJoysticks(List.of(
                Constants.Joystick.primaryJoystickId));
        switch (state) {
            case DEFAULT:
                bindButtonsDefault();
                break;
            case SYSID_TUNING:
                bindButtonsForSysid();
                break;
            case ENDGAME:
                break;
            default:
                System.out.println("Errrrrr: joystick wrong");
        }
        JoystickHandler.getInstance().init();
    }

    public static Joystick2024 getInstance() {
        return instance;
    }

    private void bindButtonsDefault() {
        Config.active.getShooter().ifPresent(s -> JoystickHandler.getInstance().bind(s));
        JoystickHandler.getInstance().bind(Config.drive());

    }

    public void setState(State state) {
        IdsWithState.state = state;
    }

    private void bindButtonsForSysid() {
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360Extended.DPadUp,
                Trigger::onTrue,
                new SimplePrintCommand("DPadUp pressed")));
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360.rb,
                Trigger::onTrue,
                new ClimberRelease(ClimberSubsystem.getInstance())));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("YrightJoystick",
                () -> JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId).getTwist(), null);
    }
}
// getX() -> joystick left x
// getY() -> joystick left y
// getZ() -> joystick right x
// getTwist() -> joystick right y
// getThrottle() -> rt
// getMagnitude() -> how close the joystick is to joystick centre
// getDirection() -> direction from -180 to 180 Degrees, joystick left
