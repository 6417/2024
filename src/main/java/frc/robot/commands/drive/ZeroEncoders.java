package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerve.SwerveDrive;
import frc.robot.subsystems.drive.swerve.SwerveModule;

public class ZeroEncoders extends ParallelCommandGroup {
    public ZeroEncoders() {
        addRequirements(SwerveDrive.getInstance());
        List<SequentialCommandGroup> commands = new ArrayList<>();
        SwerveDrive.getInstance().forEachModule((module) -> {
            SequentialCommandGroup commandGroup = new SequentialCommandGroup(new GoToHalSensor(module));
            commandGroup.addCommands(new Finetune(module, commandGroup));
            commands.add(commandGroup);
        });
        addCommands(commands.toArray(SequentialCommandGroup[]::new));
    }

    private class GoToHalSensor extends Command {
        private double currentSetPoint;
        private SwerveModule module;

        public GoToHalSensor(SwerveModule module) {
            this.module = module;
        }

        @Override
        public void initialize() {
            module.setEncoderZeroedFalse();
            module.stopAllMotors();
            module.enableLimitSwitch();
            currentSetPoint = module.getRotationEncoderTicks() + Constants.SwerveDriveConsts.zeroingSpeed;
        }

        @Override
        public void execute() {
            module.setDesiredRotationMotorTicks(currentSetPoint);
            currentSetPoint += Constants.SwerveDriveConsts.zeroingSpeed;
        }

        @Override
        public void end(boolean interrupted) {
            module.stopAllMotors();
        }

        @Override
        public boolean isFinished() {
            return module.isHalSensorTriggered();
        }
    }

    private class Finetune extends Command {
        private double currentSetPoint;
        private SwerveModule module;
        private double startingPosition;
        private Command parentSequentialCommand;

        public Finetune(SwerveModule module, Command parentSequentialCommand) {
            this.module = module;
            this.parentSequentialCommand = parentSequentialCommand;
        }

        @Override
        public void initialize() {
            module.setEncoderZeroedFalse();
            module.stopAllMotors();
            module.enableLimitSwitch();
            currentSetPoint = module.getRotationEncoderTicks()
                    - Constants.SwerveDriveConsts.zeroingSpeed * Constants.SwerveDriveConsts.finetuningZeroFactor;
            while (module.isHalSensorTriggered()) {
                module.setDesiredRotationMotorTicks(currentSetPoint);
                currentSetPoint -= Constants.SwerveDriveConsts.zeroingSpeed * Constants.SwerveDriveConsts.finetuningZeroFactor;
            }
            currentSetPoint = module.getRotationEncoderTicks();
            startingPosition = module.getRotationEncoderTicks();
        }

        @Override
        public void execute() {
            module.setDesiredRotationMotorTicks(currentSetPoint);
            currentSetPoint += Constants.SwerveDriveConsts.zeroingSpeed * Constants.SwerveDriveConsts.finetuningZeroFactor;
            if (module.isHalSensorTriggered())
                module.setRotationEncoderTicks(module.halSensorPosition);

            // TODO: Debug this if statement
            // if (Math.abs(module.getRotationEncoderTicks() - startingPosition) >
            // Constants.SwerveDrive.maxFineTuneOffsetForZeroEncodersCommand) {
            // CommandScheduler.getInstance().cancel(parentSequentialCommand);
            // SequentialCommandGroup newZeroCommandForModule = new
            // SequentialCommandGroup(new GoToHalSensor(module));
            // newZeroCommandForModule.addCommands(new Finetune(module,
            // newZeroCommandForModule));
            // CommandScheduler.getInstance().schedule(newZeroCommandForModule);
            // }
        }

        @Override
        public boolean isFinished() {
            return module.hasEncoderBeenZeroed();
        }

        @Override
        public void end(boolean interrupted) {
            module.stopAllMotors();
            module.disableLimitSwitch();
        }
    }
}
