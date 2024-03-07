package frc.robot.commands;

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

public class ShootCommand extends SequentialCommandGroup {

    ManipulatorSubsystem subsystem;
    ArmSubsystem armSubsystem;
    CommandSwerveDrivetrain swerveDrivetrain;
    static ShootCommand instance = null;
    public static boolean isSequenceActive = false;

    public ShootCommand(ManipulatorSubsystem subsystem, CommandSwerveDrivetrain swerveDrivetrain,
            ArmSubsystem armSubsystem) {
        this.subsystem = subsystem;
        this.swerveDrivetrain = swerveDrivetrain;
        this.armSubsystem = armSubsystem;

        // if (isSequenceActive) {
        // armSubsystem.shoot();
        // isSequenceActive = false;
        // return;
        // }

        // isSequenceActive = true;

        System.out.println(instance);

        addRequirements(subsystem, swerveDrivetrain, armSubsystem);
        addCommands(new InstantCommand(() -> subsystem.startShooter()),
                new InstantCommand(() -> {
                    if (instance == null) {
                        instance = this;
                    } else {
                        armSubsystem.shoot();
                        instance.cancel();
                    }
                }),
                new ParallelCommandGroup(new WaitCommand(0.2).andThen(
                        new AlignToSpeakerCommand(swerveDrivetrain)).alongWith(
                                new FunctionalCommand(
                                        () -> armSubsystem.shoot(), () -> {
                                        }, interrupted -> {
                                        }, () -> armSubsystem.atGoal()))
                        .andThen(new WaitCommand(0.02)),
                        new WaitUntilCommand(() -> subsystem.shooterAtMaxSpeed()).withTimeout(0.75)),
                new InstantCommand(() -> subsystem.intake()), new WaitCommand(.7),
                new InstantCommand(() -> subsystem.stopIntake())
                        .alongWith(new InstantCommand(() -> subsystem.stopShooter()))
                        .alongWith(new InstantCommand(() -> armSubsystem.setGoal(Constants.BACKWARD_SOFT_STOP * 360)))
                        .andThen(new InstantCommand(() -> instance = null))
        // .andThen(new InstantCommand(() -> isSequenceActive = false))
        );
    }
}
