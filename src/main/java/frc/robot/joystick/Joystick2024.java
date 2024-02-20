package frc.robot.joystick;

import java.util.List;

import javax.sound.sampled.Port;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.Xbox360;
import frc.fridowpi.joystick.joysticks.Xbox360Extended;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.commands.climber.ClimberManuel;
import frc.robot.commands.climber.ClimberRelease;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.joystick.ControllerWithState;
import frc.robot.subsystems.ClimberSubsystem;

public class Joystick2024 implements Sendable {
    private static Joystick2024 instance = new Joystick2024();

    IJoystickButtonId btnId = Xbox360.a;

    public IJoystick getPrimaryJoystick(){
        return JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId);
    }

    private Joystick2024() {
    }

    public void setup(State state) {
        JoystickHandler.getInstance().setJoystickFactory(ControllerWithState::new);
        JoystickHandler.getInstance().init(); // Don't ask, it works ;)
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
        JoystickHandler.getInstance().bind(Config.drive());
        Config.active.getShooter().ifPresent(s -> JoystickHandler.getInstance().bind(s));

    }

    public void setState(State state) {
        IdsWithState.state = state;
    }

    private void bindButtonsForSysid() {
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360.rb,
                Trigger::onTrue,
                new ClimberRelease(ClimberSubsystem.getInstance())));
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
// getX() -> joystick left x
// getY() -> joystick left y
// getThrottle() -> joystick right x
// getTwist() -> joystick right y
// getMagnitude() -> how close the joystick is to joystick centre
// getDirection() -> direction from -180 to 180 Degrees, joystick left
