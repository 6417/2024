package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BShooter;

public class ShooterSubsystem extends BShooter {

    final TalonSRX motor_left = new TalonSRX(getData().motorIds.get(0));
    final TalonSRX motor_right = new TalonSRX(getData().motorIds.get(1));
    double speeds = 0.0;

    private ShooterSubsystem() {
        motor_right.setInverted(true);
        motor_left.follow(motor_right);
    }

	@Override
    public void run() {
        if (Constants.Shooter.data.enabled) {
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
        builder.addDoubleProperty("shooterSpeeds", () -> speeds, val -> speeds = val);
    }

	@Override
	public void shoot() {
	}

	@Override
	public void enable() {
		getData().enabled = true;
	}

	@Override
	public void disable() {
		getData().enabled = false;
	}

	@Override
	public ShooterData getData() {
		return Constants.Shooter.data;
	}
}
