// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.fridowpi.motors.utils.PidValues;

public class PidTuner extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    FridoCanSparkMax testmotor;
    PidValues pid;
    double kp;
    double ki;
    double kd;
    double kf;
    double target;

    public PidTuner() {
        testmotor = new FridoCanSparkMax(14, MotorType.kBrushless);
        testmotor.configEncoder(FridoFeedBackDevice.kBuildin, 1);
        kp = 0;
        ki = 0;
        kd = 0;
        kf = 0;
        target = 0;
        pid = new PidValues(kp,ki,kd,kf);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public void refresh() {
        pid = new PidValues(kp,ki,kd,kf);
        testmotor.setPID(pid);
        testmotor.setPidTarget(target, PidType.position);
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);
        builder.addDoubleProperty("kp", () -> kp, (newkp) -> kp = newkp);
        builder.addDoubleProperty("ki", () -> ki, (newki) -> ki = newki);
        builder.addDoubleProperty("kd", () -> kd, (newkd) -> kd = newkd);
        builder.addDoubleProperty("kf", () -> kf, (newkf) -> kf = newkf);
        builder.addDoubleProperty("target", () -> target, (newtarget) -> target = newtarget);
        builder.addDoubleProperty("motorPosiiton", testmotor::getEncoderTicks, null);
        builder.addDoubleProperty("error", () -> target-testmotor.getEncoderTicks(), null);
        builder.addBooleanProperty("Set", null, _v -> refresh());
    }
}
