package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.abstraction.baseClasses.BShooter;
import frc.robot.joystick.JoystickBindings2024;

/**
 * ShooterTester
 */
public class ShooterTester extends BShooter {
	private FridolinsMotor shooterMaster;
	private FridolinsMotor shooterFollower;
	private FridolinsMotor feeder;
	private FridolinsMotor brushes;

	private double shootersSpeed = 0;
	private double feederSpeed = 0;
	private double brushesSpeed = 0;

	private final PIDController pid = new PIDController(0.0005, 0.0, 0.0);
	private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.081, 0.0020075);

	public ShooterTester() {
		shooterMaster = new FridoCanSparkMax(20, MotorType.kBrushless);
		shooterFollower = new FridoCanSparkMax(21, MotorType.kBrushless);
		feeder = new FridoCanSparkMax(22, MotorType.kBrushless);

		brushes = new FridoCanSparkMax(23, MotorType.kBrushless);

		shooterMaster.factoryDefault();
		shooterFollower.factoryDefault();
		feeder.factoryDefault();

		shooterMaster.setInverted(true);
		shooterFollower.follow(shooterMaster, DirectionType.invertMaster);

		feeder.setInverted(true);

		shooterFollower.selectPidSlot(0);
		feeder.selectPidSlot(0);
		shooterFollower.setPID(new PidValues(0, 0, 0));
		shooterMaster.setPID(new PidValues(0, 0, 0));

		shooterFollower.configEncoder(FridoFeedBackDevice.kBuildin, getData().countsPerRevolution);
		feeder.configEncoder(FridoFeedBackDevice.kBuildin, getData().countsPerRevolution);

		shooterFollower.setIdleMode(IdleMode.kCoast);
		shooterMaster.setIdleMode(IdleMode.kCoast);
		feeder.setIdleMode(IdleMode.kCoast);


		((CANSparkMax) shooterFollower).setSmartCurrentLimit(20);
		((CANSparkMax) shooterMaster).setSmartCurrentLimit(20);
		((CANSparkMax) feeder).setSmartCurrentLimit(40);

		((CANSparkMax) shooterFollower).enableVoltageCompensation(12);
		((CANSparkMax) shooterMaster).enableVoltageCompensation(12);
		((CANSparkMax) feeder).enableVoltageCompensation(12);

		pid.setTolerance(50);
		pid.setIntegratorRange(-0.015, 0.015);
	}

	@Override
	public void shoot(IShooterConfig configuration) {
	}

	@Override
	public void setShooterSpeedPercent(double speed) {
	}

	@Override
	public void run() {
	}

	@Override
	public void enable() {
	}

	@Override
	public void disable() {
	}

	@Override
	public double getSpeedPercent() {
		return 0;
	}

	@Override
	public ShooterData getData() {
		return null;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("shooterSpeed", () -> shootersSpeed, val -> shootersSpeed = val);
		builder.addDoubleProperty("feederSpeed", () -> feederSpeed, val -> feederSpeed = val);
		builder.addDoubleProperty("brushesSpeed", () -> brushesSpeed, val -> brushesSpeed = val);

		builder.addBooleanProperty("setShooter", null, val -> shooterMaster.set(val? shootersSpeed : 0));
		builder.addBooleanProperty("setFeeder", null, val -> feeder.set(val? feederSpeed : 0));
		builder.addBooleanProperty("setBrushes", null, val -> brushes.set(val? brushesSpeed : 0));

		builder.addDoubleProperty("velocityPid", pid::getSetpoint, val -> pid.setSetpoint(val));
	}

	@Override
	public List<Binding> getMappings() {
		JoystickBindings2024.tmp_bindings.clear();
		JoystickBindings2024.quickBindWhileHeld(
				Logitech.a, () -> shooterMaster.set(shootersSpeed));
		return JoystickBindings2024.tmp_bindings;
	}

	@Override
	public void stopMotors() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'stopMotors'");
	}
}
