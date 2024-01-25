package frc.robot.subsystems.drive.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
    private TalonFX fahrmotor = new TalonFX(3);
    private CANSparkMax drehmotor = new CANSparkMax(1, MotorType.kBrushless);
}
