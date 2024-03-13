package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(ManipulatorSubsystem subsystem,
            ArmSubsystem armSubsystem) {
        InstantCommand startShooter = new InstantCommand(() -> subsystem.startShooter());
        InstantCommand stopShooter = new InstantCommand(() -> subsystem.stopShooter());
        Command alignArm = new FunctionalCommand(
                () -> armSubsystem.shoot(), () -> {
                }, interrupted -> {
                }, () -> armSubsystem.atGoal()).withTimeout(5); // used in aimCommands
        Command checkShooterSpeed = new WaitUntilCommand(() -> subsystem.shooterAtMaxSpeed()).withTimeout(1.5);

        InstantCommand intakeToShooter = new InstantCommand(() -> subsystem.intake());
        InstantCommand stopIntake = new InstantCommand(() -> subsystem.stopIntake());
        InstantCommand resetArm = new InstantCommand(
                () -> armSubsystem.setGoal(Constants.BACKWARD_SOFT_STOP * 360));
        addRequirements(subsystem, armSubsystem);
        addCommands(startShooter, alignArm.alongWith(checkShooterSpeed), new WaitCommand(0.5), intakeToShooter,
                new WaitCommand(0.9),
                new ParallelCommandGroup(resetArm, stopShooter, stopIntake));
    }

}
