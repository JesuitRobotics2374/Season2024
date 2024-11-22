package frc.robot.commands.unused;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SequentialApproach extends SequentialCommandGroup {
    public void AutomaticAmp(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {

        // InstantCommand align = new InstantCommand(() -> AlignDynamic());

    }
}
