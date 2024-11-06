package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem.DistanceAndAngle;

public class ApproachTag extends InstantCommand {

    public ApproachTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {

        DistanceAndAngle d = visionSubsystem.getTagDistanceAndAngle(tag_id);

        Command align = new AlignDynamic(drivetrain, d.getTheta());
        Command drive = new DriveDynamic(drivetrain, visionSubsystem, tag_id);

        SequentialCommandGroup approach = new SequentialCommandGroup(align, drive);

        approach.schedule();

    }
}
