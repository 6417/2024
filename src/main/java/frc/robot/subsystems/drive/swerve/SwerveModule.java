package frc.robot.subsystems.drive.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private static SwerveModule instance = null;

    private TalonFX fahrmotor = new TalonFX(2);
    private CANSparkMax drehmotor = new CANSparkMax(3, MotorType.kBrushless);

    private PIDController pidController = new PIDController(0.001, 0, 0);
    AnalogEncoder absEncoder = new AnalogEncoder(0);

    private SwerveModule() {
        pidController.setSetpoint(0.0);
    }

    public void periodic() {
        var curPos = absEncoder.getAbsolutePosition();
        fahrmotor.set(0);
        drehmotor.set(pidController.calculate(curPos));
        // drehmotor.set(0.2);
    }

    public static SwerveModule getInstance() {
        if (instance == null) {
            instance = new SwerveModule();
        }
        return instance;
    }

    private double[] pos = { 0, 0, 0 };

    @Override
    public void initSendable(SendableBuilder builder) {
        for (int i = 0; i <= 2; i++) {
            final int j = i;
            builder.addDoubleProperty("Position" + i, () -> pos[j],
                    (val) -> pos[j] = val);
            builder.addBooleanProperty("pos" + i + "toggle", () -> false,
                    (isActive) -> pidController.setSetpoint(pos[j]));
        }
        builder.addDoubleProperty("P", pidController::getP, (val) -> pidController.setP(val));
        builder.addDoubleProperty("I", pidController::getI, (val) -> pidController.setI(val));
        builder.addDoubleProperty("D", pidController::getD, (val) -> pidController.setD(val));
        builder.addDoubleProperty("calculatedCurPos", () -> pidController.calculate(absEncoder.getAbsolutePosition()),
                null);
        builder.addDoubleProperty("curentPos", absEncoder::getAbsolutePosition, null);
    }
}
