package frc.robot.joystick;

import java.util.List;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.XBoxJoystick;
import frc.fridowpi.joystick.joysticks.Xbox360;
import frc.fridowpi.joystick.joysticks.Xbox360Extended;
import frc.robot.Constants;
import frc.robot.commands.SimplePrintCommand;
import frc.robot.commands.climber.ClimberManuel;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Tankdrive;

public class Joystick2024 {
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
        JoystickHandler.getInstance().bind(ShooterSubsystem.getInstance());
        JoystickHandler.getInstance().bind(Tankdrive.getInstance());
    }

    public void setState(State state) {
        IdsWithState.state = state;
    }

    private void bindButtonsForSysid() {
        IdsWithState.state = State.SYSID_TUNING;
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360Extended.DPadDownLeft,
                Trigger::onTrue,
                new SimplePrintCommand("DPadDownLeft pressed")));
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360Extended.DPadDown,
                Trigger::onTrue,
                new SimplePrintCommand("DPadDown pressed")));
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360Extended.DPadRight,
                Trigger::onTrue,
                new SimplePrintCommand("DPadRight pressed")));
        JoystickHandler.getInstance().bind(new Binding(
                Constants.Joystick.primaryJoystickId,
                Xbox360Extended.DPadDownRight,
                Trigger::onTrue,
                new SimplePrintCommand("DPadDownRight pressed")));
        ;

        // if (Controls.joystick.getAButtonPressed()) {
        // SignalLogger.start();
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
        // SignalLogger.start();
        // } else if (Controls.joystick.getYButtonPressed()) {
        // SignalLogger.start();
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
        // SignalLogger.start();
        // } else if (Controls.joystick.getXButtonPressed()) {
        // SignalLogger.start();
        // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
        // SignalLogger.start();
        // } else if (Controls.joystick.getBButtonPressed()) {
        // SignalLogger.start();
        // drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
        // gnalLogger.start();
        // } else if (Controls.joystick.getAButtonReleased() ||
        // Controls.joystick.getBButtonReleased() ||
        // Controls.joystick.getXButtonReleased() ||
        // Controls.joystick.getYButtonReleased()) {
        // SignalLogger.stop();
        // CommandScheduler.getInstance().cancelAll();
        // drive.brake();
        // SignalLogger.stop();
        // }
    }
}
