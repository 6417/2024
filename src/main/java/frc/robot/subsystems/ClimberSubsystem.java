// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;
    
    private FridolinsMotor seilZiehMotorLinks = new FridoCanSparkMax(Constants.Climber.seilZiehMotorLinks, MotorType.kBrushless);
    private FridolinsMotor seilZiehMotorRechts = new FridoCanSparkMax(Constants.Climber.seilZiehMotorRechts, MotorType.kBrushless);
    
    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {

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
}
