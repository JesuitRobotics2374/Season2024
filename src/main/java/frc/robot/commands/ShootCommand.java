package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class ShootCommand extends SequentialCommandGroup {

    ManipulatorSubsystem subsystem;
    CommandSwerveDrivetrain swerveDrivetrain;

    public ShootCommand(ManipulatorSubsystem subsystem, CommandSwerveDrivetrain swerveDrivetrain) {
        this.subsystem = subsystem;
        this.swerveDrivetrain = swerveDrivetrain;
        addRequirements(subsystem, swerveDrivetrain);
        addCommands(new InstantCommand(() -> subsystem.startShooter()),
                new ParallelCommandGroup(new AlignToSpeakerCommand(swerveDrivetrain), new WaitCommand(1.5)),
                new InstantCommand(() -> subsystem.startIntake()), new WaitCommand(.5),
                new InstantCommand(() -> subsystem.stopIntake())
                        .alongWith(new InstantCommand(() -> subsystem.stopShooter())));
    }
}
