// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public static FridolinsMotor seilZiehMotorLinks = new FridoCanSparkMax(Constants.Climber.seilZiehMotorLinks,
            MotorType.kBrushless);
    public static FridolinsMotor seilZiehMotorRechts = new FridoCanSparkMax(Constants.Climber.seilZiehMotorRechts,
            MotorType.kBrushless);
    private Servo federLoslassMotorLinks = new Servo(Constants.Climber.federLoslassMotorLinks);
    private Servo federLoslassMotorRechts = new Servo(Constants.Climber.federLoslassMotorRechts);

    /** Creates a new ClimberSubsystem. */
    private ClimberSubsystem() {
        seilZiehMotorRechts.follow(seilZiehMotorLinks, DirectionType.followMaster);
        seilZiehMotorLinks.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }

    public void federnFreilassenOderFangen() {
        federLoslassMotorLinks.setAngle(federLoslassMotorLinks.getAngle() + 180);
        federLoslassMotorRechts.setAngle(federLoslassMotorRechts.getAngle() + 180);
    }
}
