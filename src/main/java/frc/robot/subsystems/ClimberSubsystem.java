// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public FridolinsMotor seilZiehMotorLinks = new FridoFalcon500(Constants.Climber.seilZiehMotorLinks);
    public FridolinsMotor seilZiehMotorRechts = new FridoFalcon500(Constants.Climber.seilZiehMotorRechts);
    public Servo federLoslassMotorLinks = new Servo(Constants.Climber.federLoslassMotorLinks);
    
    public Servo federLoslassMotorRechts = new Servo(Constants.Climber.federLoslassMotorRechts);
    
    /** Creates a new ClimberSubsystem. */
    private ClimberSubsystem() {
        seilZiehMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
        seilZiehMotorRechts.setPID(Constants.Climber.pidValuesSlot0);
        seilZiehMotorRechts.follow(seilZiehMotorLinks, DirectionType.followMaster);
        seilZiehMotorLinks.setIdleMode(IdleMode.kBrake);
        // federLoslassMotorLinks.setAngle(0);
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
    }
}
