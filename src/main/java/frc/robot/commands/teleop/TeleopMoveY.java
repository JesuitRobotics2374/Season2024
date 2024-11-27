package frc.robot.commands.teleop;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

/**
 * DriveDynamic - Moves the robot forward by a specified distance.
 */
public class TeleopMoveY extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int tag_id;
    private double distanceFromTagAlign;
    private double moveSpeed = 0.5; // change this later, possibly make dynamic or keep constant

    public TeleopMoveY(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.tag_id = tag_id;

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        if (visionSubsystem.canSeeTag(tag_id)) {
            distanceFromTagAlign = visionSubsystem.getTagPose3d(tag_id).getY();
            if (distanceFromTagAlign > 0) {
                moveSpeed *= -1;
            }
        } else
            cancel(); // Check if this works
    }

    @Override
    public void execute() {
        if (visionSubsystem.canSeeTag(tag_id)) {
            distanceFromTagAlign = visionSubsystem.getTagPose3d(tag_id).getY();
        }
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(moveSpeed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(distanceFromTagAlign) < 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement Y complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(0));
    }

}