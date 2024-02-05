// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    private FridolinsMotor seilZiehMotorLinks = new FridoCanSparkMax(Constants.Climber.seilZiehMotorLinks,
            MotorType.kBrushless);
    private FridolinsMotor seilZiehMotorRechts = new FridoCanSparkMax(Constants.Climber.seilZiehMotorRechts,
            MotorType.kBrushless);
    private Servo federLoslassMotorLinks = new Servo(Constants.Climber.federLoslassMotorLinks);
    private Servo federLoslassMotorRechts = new Servo(Constants.Climber.federLoslassMotorRechts);

    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {

    }

    @Override
    public void periodic() {
        if (Math.abs(seilZiehMotorLinks.getEncoderTicks()
                - seilZiehMotorRechts.getEncoderTicks()) <= Constants.Climber.toleranzDerHoheDerMotoren) {
            // Die Werte sind in der Nähe.
            seilZiehMotorLinks.set(0.2);
            seilZiehMotorRechts.set(0.2);
        } else if (seilZiehMotorLinks.getEncoderTicks() < seilZiehMotorRechts.getEncoderTicks()) {
            seilZiehMotorLinks.set(0.1);
            // Die Werte sind nicht in der Nähe.
        } else if (seilZiehMotorLinks.getEncoderTicks() > seilZiehMotorRechts.getEncoderTicks()) {
            seilZiehMotorRechts.set(0.1);
        }
        // This method will be called once per scheduler run
    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }

    public void seilZiehMotorenFahren() {
        if (Math.abs(seilZiehMotorLinks.getEncoderTicks()
                - seilZiehMotorRechts.getEncoderTicks()) <= Constants.Climber.toleranzDerHoheDerMotoren) {
            // Die Werte sind in der Nähe von einander.
            seilZiehMotorLinks.set(0.2);
            seilZiehMotorRechts.set(0.2);
        } else if (seilZiehMotorLinks.getEncoderTicks() < seilZiehMotorRechts.getEncoderTicks()) {
            seilZiehMotorLinks.set(0.1);
            // Die Werte sind nicht in der Nähe.
        } else if (seilZiehMotorLinks.getEncoderTicks() > seilZiehMotorRechts.getEncoderTicks()) {
            seilZiehMotorRechts.set(0.1);
        }
    }
}
