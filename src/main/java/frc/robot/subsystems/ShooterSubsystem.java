package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.abstraction.baseClasses.BShooter;

public class ShooterSubsystem extends BShooter {

    final TalonSRX motor_left = new TalonSRX(22); // TODO: impove
    final TalonSRX motor_right = new TalonSRX(23);
    double speeds = 0.0;

	boolean enabled = true;

    private ShooterSubsystem() {
        motor_right.setInverted(true);
        motor_left.follow(motor_right);
    }

	@Override
    public void run() {
        if (enabled) {
            motor_right.set(TalonSRXControlMode.PercentOutput, speeds);
        } else {
            motor_right.set(TalonSRXControlMode.Disabled, 0.0);
        }
    }

	@Override
	public void setSpeedPercent(double speed) {
		if (speed > 1.0) {
			System.out.println("Speed too high: " + speed);
			speed = 1.0;
		} else if (speed < -1.0) {
			System.out.println("Speed too low: " + speed);
			speed = -1.0;
		}
        speeds = speed;
    }

	@Override
	public double getSpeedPercent() {
		return speeds;
	}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("shooterSpeeds", () -> speeds, (val) -> speeds = val);
    }

	@Override
	public void shoot() {
	}

	@Override
	public void enable() {
		enabled = true;
	}

	@Override
	public void disable() {
		enabled = false;
	}
}
