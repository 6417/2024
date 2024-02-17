// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.MatchInfoData;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridoServoMotor;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.Constants;
import frc.robot.Controls;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public FridolinsMotor seilZiehMotorLinks = new FridoFalcon500(Constants.Climber.seilZiehMotorLinks);
    public FridolinsMotor seilZiehMotorRechts = new FridoFalcon500(Constants.Climber.seilZiehMotorRechts);
    public FridoServoMotor federLoslassMotorLinks = new FridoServoMotor(Constants.Climber.federLoslassMotorLinks);
    public FridoServoMotor federLoslassMotorRechts = new FridoServoMotor(Constants.Climber.federLoslassMotorRechts);
    
    /** Creates a new ClimberSubsystem. */
    private ClimberSubsystem() {
        seilZiehMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
        seilZiehMotorRechts.setPID(Constants.Climber.pidValuesSlot0);
        seilZiehMotorRechts.follow(seilZiehMotorLinks, DirectionType.followMaster);
        seilZiehMotorLinks.setIdleMode(IdleMode.kBrake);
        federLoslassMotorLinks.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
        federLoslassMotorLinks.setMaxAngle(130);
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
