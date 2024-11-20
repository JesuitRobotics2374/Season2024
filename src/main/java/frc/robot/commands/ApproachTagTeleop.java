package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.helpers.SubsystemAction;
import frc.robot.commands.teleop.TeleopMoveX;
import frc.robot.commands.teleop.TeleopMoveY;
import frc.robot.commands.teleop.TeleopRotate;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VacummSubystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class ApproachTagTeleop extends InstantCommand {

    public ApproachTagTeleop(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id,
            VacummSubystem vac, ArmSubsystem arm) {

        Command moveX = new TeleopMoveX(drivetrain, visionSubsystem, tag_id);
        Command moveY = new TeleopMoveY(drivetrain, visionSubsystem, tag_id);
        Command rotate = new TeleopRotate(drivetrain, visionSubsystem, tag_id);

        Command outtake = new SubsystemAction(vac, arm, "outtake");
        Command timer = new WaitCommand(0.6);
        Command stop = new SubsystemAction(vac, arm, "stop");

        ParallelCommandGroup approach = new ParallelCommandGroup(rotate, moveY, moveX);

        SequentialCommandGroup outtakeSequence = new SequentialCommandGroup(outtake, timer, stop);

        approach.schedule();
        outtakeSequence.schedule();

    }
}
