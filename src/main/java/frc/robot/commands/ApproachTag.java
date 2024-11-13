package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VacummSubystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem.DistanceAndAngle;

public class ApproachTag extends InstantCommand {

    public ApproachTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id,
            VacummSubystem vac, ArmSubsystem arm) {

        // DistanceAndAngle d = visionSubsystem.getTagDistanceAndAngle(tag_id);

        Command intake = new SubsystemAction(vac, arm, "intake");
        Command seek = new DriveAndSeek(drivetrain, visionSubsystem, tag_id);
        Command squareUp = new DriveDynamicY(drivetrain, visionSubsystem, tag_id, 1, 3);
        Command drive = new DriveDynamic(drivetrain, visionSubsystem, tag_id, 1.4, 36);
        Command squareUpClose = new DriveDynamicY(drivetrain, visionSubsystem, tag_id, 0.4, 0.5);
        Command lower = new SubsystemAction(vac, arm, "lower");
        Command driveClose = new DriveDynamic(drivetrain, visionSubsystem, tag_id, 0.6, 0);
        Command outtake = new SubsystemAction(vac, arm, "outtake");
        Command timer = new WaitCommand(0.6);
        Command stop = new SubsystemAction(vac, arm, "stop");

        SequentialCommandGroup approach = new SequentialCommandGroup(intake, seek, squareUp, drive, squareUpClose,
                lower, driveClose, outtake, timer, stop);

        approach.schedule();

    }
}
