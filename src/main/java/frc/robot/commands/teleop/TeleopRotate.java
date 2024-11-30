package frc.robot.commands.teleop;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class TeleopRotate extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int tag_id;
    private double tagAlignAngle; // in degrees
    private double turnSpeed = 1; // change this later, possibly make dynamic or keep constant, in RADIANS / SEC

    public TeleopRotate(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.tag_id = tag_id;

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        if (visionSubsystem.canSeeTag(tag_id)) {
            tagAlignAngle = visionSubsystem.getTagDistanceAndAngle(tag_id).getTheta();
            if (tagAlignAngle > 0) {
                turnSpeed *= -1;
            }
        } else
            cancel(); // Check if this works
    }

    @Override
    public void execute() {
        if (visionSubsystem.canSeeTag(tag_id)) {
            tagAlignAngle = visionSubsystem.getTagDistanceAndAngle(tag_id).getTheta(); // in degrees
            drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(turnSpeed));
        }
    }

    @Override
    public boolean isFinished() {
        return tagAlignAngle > -7 && tagAlignAngle < 7; // in degrees
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Rotation Complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(0));
        ;
    }

}