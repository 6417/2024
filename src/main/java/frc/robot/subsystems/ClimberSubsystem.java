// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;
import frc.robot.commands.ClimberPid;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public static FridolinsMotor seilZiehMotorLinks = new FridoCanSparkMax(Constants.Climber.seilZiehMotorLinks,
            MotorType.kBrushless);
    public static FridolinsMotor seilZiehMotorRechts = new FridoCanSparkMax(Constants.Climber.seilZiehMotorRechts,
            MotorType.kBrushless);
    private Servo federLoslassMotorLinks = new Servo(Constants.Climber.federLoslassMotorLinks);
    private Servo federLoslassMotorRechts = new Servo(Constants.Climber.federLoslassMotorRechts);

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

    
    public void federnFreilassenOFangen() {
        federLoslassMotorLinks.setAngle(federLoslassMotorLinks.getAngle() + 180);
        federLoslassMotorRechts.setAngle(federLoslassMotorRechts.getAngle() + 180);
    }

    private Command[] commandliste = new Command[Constants.Climber.anzFahrAbschnitte];

    public void climberBedienen() {
        var b = Constants.Climber.ausfahrBereich;
        var n = Constants.Climber.anzFahrAbschnitte;
        for (int i = 0; i < n; i++) {
            commandliste[i] = new ClimberPid(b / n * i);
        }
    }
    // public void seilZiehMotorenEinfahren() {
        // if (Math.abs(seilZiehMotorLinks.getEncoderTicks()
        // - seilZiehMotorRechts.getEncoderTicks()) <=
        // Constants.Climber.toleranzDerHoheDerMotoren) {
            // // Die Werte sind in der Nähe von einander.
    // seilZiehMotorLinks.set(Constants.Climber.raufZiehSpeed);
    // seilZiehMotorRechts.set(Constants.Climber.raufZiehSpeed);
    // } else if (seilZiehMotorLinks.getEncoderTicks() <
    // seilZiehMotorRechts.getEncoderTicks()) {
        // seilZiehMotorLinks.set(Constants.Climber.raufZiehSpeed);
        // // Die Werte sind nicht in der Nähe.
        // } else if (seilZiehMotorLinks.getEncoderTicks() >
        // seilZiehMotorRechts.getEncoderTicks()) {
            // seilZiehMotorRechts.set(Constants.Climber.raufZiehSpeed);
            // }
            // }


            // public void seilZiehMotorenPID() {
        
            //     seilZiehMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
            //     seilZiehMotorRechts.setPID(Constants.Climber.pidValuesSlot0);
            //     seilZiehMotorLinks.setPidTarget(Constants.Climber.ausfahrBereich / Constants.Climber.anzFahrAbschnitte,
            //             PidType.position);
            //     seilZiehMotorRechts.setPidTarget(Constants.Climber.ausfahrBereich / Constants.Climber.anzFahrAbschnitte,
            //             PidType.position);
        
            //     if (seilZiehMotorLinks.pidAtTarget() && seilZiehMotorRechts.pidAtTarget()) {
            //         seilZiehMotorLinks.setPidTarget(Constants.Climber.ausfahrBereich / Constants.Climber.anzFahrAbschnitte,
            //                 PidType.position);
            //         seilZiehMotorRechts.setPidTarget(Constants.Climber.ausfahrBereich / Constants.Climber.anzFahrAbschnitte,
            //                 PidType.position);
            //     }
        
            // }
        }
        