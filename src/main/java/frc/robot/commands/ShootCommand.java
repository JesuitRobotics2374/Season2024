package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class ShootCommand extends SequentialCommandGroup {

    ManipulatorSubsystem subsystem;
    ArmSubsystem armSubsystem;
    CommandSwerveDrivetrain swerveDrivetrain;

    public ShootCommand(ManipulatorSubsystem subsystem, CommandSwerveDrivetrain swerveDrivetrain,
            ArmSubsystem armSubsystem) {
        this.subsystem = subsystem;
        this.swerveDrivetrain = swerveDrivetrain;
        this.armSubsystem = armSubsystem;
        addRequirements(subsystem, swerveDrivetrain, armSubsystem);
        addCommands(new InstantCommand(() -> subsystem.startShooter()),
                new ParallelCommandGroup(
                        new AlignToSpeakerCommand(swerveDrivetrain).alongWith(new FunctionalCommand(
                                () -> armSubsystem.shoot(), null, null, () -> armSubsystem.atGoal())),
                        new WaitCommand(1)),
                new InstantCommand(() -> subsystem.intake()), new WaitCommand(.5),
                new InstantCommand(() -> subsystem.stopIntake())
                        .alongWith(new InstantCommand(() -> subsystem.stopShooter())));
    }
}
