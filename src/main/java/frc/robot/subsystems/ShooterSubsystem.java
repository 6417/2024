package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase implements Sendable {

    TalonSRX motor_left = new TalonSRX(22);
    TalonSRX motor_right = new TalonSRX(23);
    double speeds = 0.0;

    public ShooterSubsystem() {
        motor_left.follow(motor_right);
    }

    public void changeMotorSpeed(double perCentChange) {
        speeds += perCentChange;
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
