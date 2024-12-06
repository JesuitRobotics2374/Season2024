package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DriveAndSeek;
import frc.robot.commands.auto.DriveDynamic;
import frc.robot.commands.auto.DriveDynamicY;
import frc.robot.commands.auto.OriginToStatic;
import frc.robot.commands.helpers.SubsystemAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VacummSubystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class ApproachTagAuto extends InstantCommand {

    public ApproachTagAuto(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id,
            VacummSubystem vac, ArmSubsystem arm) {

        // DistanceAndAngle d = visionSubsystem.getTagDistanceAndAngle(tag_id);

        Command blind = new BackUntilCanSeeTag(drivetrain, visionSubsystem);
        Command align = new InstantCommand(() -> {
            drivetrain.alignToVision();
        });
        Command intake = new SubsystemAction(vac, arm, "intake");
        Command staticNav = new OriginToStatic(drivetrain, visionSubsystem, tag_id);
        Command seek = new DriveAndSeek(drivetrain, visionSubsystem, tag_id);
        Command squareUp = new DriveDynamicY(drivetrain, visionSubsystem, tag_id, 1, 0.15);
        Command drive = new DriveDynamic(drivetrain, visionSubsystem, tag_id, 1.4, 1.5);
        Command squareUpClose = new DriveDynamicY(drivetrain, visionSubsystem, tag_id, 0.3, 0.01);
        Command lower = new SubsystemAction(vac, arm, "lower");
        Command driveClose = new DriveDynamic(drivetrain, visionSubsystem, tag_id, 0.2, 0.3);
        Command outtake = new SubsystemAction(vac, arm, "outtake");
        Command timer = new WaitCommand(0.6);
        Command stop = new SubsystemAction(vac, arm, "stop");

        SequentialCommandGroup approach = new SequentialCommandGroup(blind, align, intake, staticNav, seek, 
        squareUp, drive, squareUpClose, lower, driveClose, outtake, timer, stop);

        approach.schedule();

    }
}
