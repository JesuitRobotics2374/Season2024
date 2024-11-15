package frc.robot.commands.unused;

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
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class AutoShootCommand extends SequentialCommandGroup {

    ManipulatorSubsystem subsystem;
    ArmSubsystem armSubsystem;
    CommandSwerveDrivetrain swerveDrivetrain;
    // public static ShootCommand instance = null;
    public static boolean isSequenceActive = false;

    public AutoShootCommand(ManipulatorSubsystem subsystem, CommandSwerveDrivetrain swerveDrivetrain,
            ArmSubsystem armSubsystem) {
        this.subsystem = subsystem;
        this.swerveDrivetrain = swerveDrivetrain;
        this.armSubsystem = armSubsystem;

        /// Commands

        InstantCommand startShooter = new InstantCommand(() -> subsystem.startShooter());

        Command alignDrivetrain = new AlignToSpeakerCommand(swerveDrivetrain); // used in aimCommands
        Command alignArm = new FunctionalCommand(
                () -> armSubsystem.shoot(), () -> {
                }, interrupted -> {
                }, () -> armSubsystem.atGoal()); // used in aimCommands

        Command aimCommands = new ParallelCommandGroup(alignDrivetrain, alignArm)
                .andThen(new WaitCommand(0.5));

        Command checkShooterSpeed = new WaitUntilCommand(() -> subsystem.shooterAtMaxSpeed()).withTimeout(0.75);

        InstantCommand intakeToShooter = new InstantCommand(() -> subsystem.intake());

        InstantCommand stopIntake = new InstantCommand(() -> subsystem.stopIntake());
        InstantCommand stopShooter = new InstantCommand(() -> subsystem.stopShooter());
        InstantCommand resetArm = new InstantCommand(
                () -> armSubsystem.setGoal(Constants.BACKWARD_SOFT_STOP * 360));
        InstantCommand nullifyInstance = new InstantCommand(() -> {
            // instance = null;
            isSequenceActive = false;
        });

        // InstantCommand cancel = new InstantCommand(() -> {
        // // System.err.println(instance);
        // // if (false) {
        // // instance = this;
        // // System.err.println("New Shot");
        // // } else {
        // subsystem.intake(); // intake to shooter
        // System.err.println("Shot Cancelled");
        // // instance.cancel();
        // // instance = null;
        // // }
        // });

        // System.err.println(instance);

        isSequenceActive = true;

        addRequirements(subsystem, swerveDrivetrain, armSubsystem);
        addCommands(startShooter,
                new ParallelCommandGroup(aimCommands, checkShooterSpeed).withTimeout(2.5),
                intakeToShooter, new WaitCommand(.9),
                new ParallelCommandGroup(resetArm, stopShooter, stopIntake),
                nullifyInstance);
    }
}
