package frc.robot.subsystems.drive.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.abstraction.interfaces.ISwerveModule;

public class SwerveModulePhoenixSparkMax  implements ISwerveModule {
    TalonFX driveMotor;
    CANSparkMax turnMotor;
    AnalogEncoder absEncoder;

    public SwerveModulePhoenixSparkMax(int driveMotorId, int turnMotorId, int encoderChannel) {
        driveMotor = new TalonFX(driveMotorId);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        absEncoder = new AnalogEncoder(encoderChannel);
    }
}
