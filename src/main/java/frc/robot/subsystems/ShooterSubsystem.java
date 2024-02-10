package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.JoystickBindable;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.XBoxJoystick;
import frc.fridowpi.joystick.joysticks.Xbox360;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.joystick.ControllerWithState;
import frc.robot.joystick.IdsWithState;

public class ShooterSubsystem extends SubsystemBase implements Sendable, JoystickBindable {
    /* Ideal values:
     *  - Intake        : -0.5
     *  - Shoot AMP     : 0.3
     *  - Shoot SPEAKER : 0.8 to 0.9
     */
    static ShooterSubsystem instance;

    final TalonSRX motor_left = new TalonSRX(22);
    final TalonSRX motor_right = new TalonSRX(23);
    double speeds = 0.0;
    double speedChange = 0.01;

    private ShooterSubsystem() {
        motor_right.setInverted(true);
        motor_left.follow(motor_right);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    public void changeMotorSpeed(int direction) {
        speeds += direction * speedChange;
    }

    public void setMotorSpeed(double speed) {
        speeds = speed;
    }

    public void run(boolean enabled) {
        if (enabled) {
            motor_right.set(TalonSRXControlMode.PercentOutput, speeds);
        } else {
            motor_right.set(TalonSRXControlMode.Disabled, 0.0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("shooterSpeeds", () -> speeds, (val) -> speeds = val);
        builder.addDoubleProperty("speedChange", () -> speedChange, (val) -> speedChange = val);
    }

    @Override
    public List<Binding> getMappings() {
        // Shooter
        // var id = Constants.Joystick.primaryJoystickId;
        // var joystick = JoystickHandler.getInstance().getJoystick(id);
        // return List.of(
        //     new Binding(id, IdsWithState.DPadUp, null, null)
        // );
        // if (Controls.joystick.getPOV() == 0) {
        //     shooter.changeMotorSpeed(1);
        // } else if (Controls.joystick.getPOV() == 180) {
        //     shooter.changeMotorSpeed(-1);
        // }
        // if (Controls.joystick.getYButtonPressed()) {
        //     shooter.setMotorSpeed(Constants.Shooter.OptimalSpeakerSpeed);
        // } else if (Controls.joystick.getXButtonPressed()) {
        //     shooter.setMotorSpeed(Constants.Shooter.OptimalAmpSpeed);
        // } else if (Controls.joystick.getBButtonPressed()) {
        //     shooter.setMotorSpeed(0.0);
        // }
        return List.of();
    }
}
