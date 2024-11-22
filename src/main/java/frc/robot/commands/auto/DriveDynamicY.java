package frc.robot.commands.auto;

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
public class DriveDynamicY extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int tag_id;
    // private ProfiledPIDController controller;
    private double currentPositionMeters;
    private double relativeDistanceMeters; // Desired distance to move (in meters)
    private double targetPositionMeters; // Final target position for the robot

    private double moveSpeed;
    private double threshold;

    private boolean done;
    private double fdist;

    /**
     * DriveDynamic Constructor
     * 
     * @param drivetrain             The swerve drivetrain subsystem
     * @param relativeDistanceMeters The desired distance to move forward (in
     *                               meters)
     */
    public DriveDynamicY(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id,
            double speed, double threshold) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.tag_id = tag_id;
        this.moveSpeed = speed;
        this.threshold = threshold;

        // this.relativeDistanceMeters =
        // visionSubsystem.getTagDistanceAndAngle(3).getDistanceMeters() - 0.1;

        // Initialize the ProfiledPIDController with PID constants and constraints
        // controller = new ProfiledPIDController(Constants.P_ARM_PID_P,
        // Constants.P_ARM_PID_I, Constants.P_ARM_PID_D,
        // new Constraints(0.2, 0.1)); // Velocity and Acceleration constraints

        // controller.setTolerance(0.02, 0.02); // Tolerance for position and velocity
        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        // Get the current robot position in meters
        currentPositionMeters = drivetrain.getState().Pose.getTranslation().getX();

        double dist = visionSubsystem.getTagDistanceAndAngle(tag_id).getDistanceMeters();
        double th = visionSubsystem.getTagDistanceAndAngle(tag_id).getTheta();
        double signum = Math.abs(th) / th;

        drivetrain.setControl(
                new SwerveRequest.RobotCentric().withVelocityY(-moveSpeed * signum));

        // return !visionSubsystem.canSeeTag(tag_id);

        System.out.println(th);
        if (Math.abs(th) <= threshold || !visionSubsystem.canSeeTag(tag_id)) {
            fdist = th;
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("Command " + (interrupted ? "interrupted" : "completed") + ". Final robot position: "
                + drivetrain.getState().Pose.getTranslation().getX() + " meters.");
        System.out.println("Final: " + visionSubsystem.getTagDistanceAndAngle(tag_id).getDistance());
    }

    public double getGoal() {
        return targetPositionMeters;
    }

    public double getRelativeDistance() {
        return relativeDistanceMeters;
    }

}