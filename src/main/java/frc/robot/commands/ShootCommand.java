package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
                                () -> armSubsystem.shoot(), () -> {
                                }, interrupted -> {
                                }, () -> armSubsystem.atGoal())).andThen(new WaitCommand(0.1)),
                        new WaitUntilCommand(() -> subsystem.shooterAtMaxSpeed()).withTimeout(1.2)),
                new InstantCommand(() -> subsystem.intake()), new WaitCommand(.6),
                new InstantCommand(() -> subsystem.stopIntake())
                        .alongWith(new InstantCommand(() -> subsystem.stopShooter()))
                        .alongWith(new InstantCommand(() -> armSubsystem.setGoal(-0.242 * 360))));
    }
}
