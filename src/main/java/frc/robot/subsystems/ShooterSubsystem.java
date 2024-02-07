package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    /* Ideal values:
     *  - Intake        : -0.5
     *  - Shoot AMP     : 0.3
     *  - Shoot SPEAKER : 0.8 to 0.9
     */
    static ShooterSubsystem instance = new ShooterSubsystem();

    final TalonSRX motor_left = new TalonSRX(22);
    final TalonSRX motor_right = new TalonSRX(23);
    double speeds = 0.0;

    private ShooterSubsystem() {
        motor_right.setInverted(true);
        motor_left.follow(motor_right);
    }

    public static ShooterSubsystem getInstance() {
        return instance;
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
    }
}
