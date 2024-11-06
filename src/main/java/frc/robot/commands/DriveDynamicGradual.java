package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import frc.robot.Constants;

/**
 * DriveDynamic - Moves the robot forward by a specified distance.
 */
public class DriveDynamicGradual extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int tag_id;
    // private ProfiledPIDController controller;
    private double currentPositionMeters;
    private double relativeDistanceMeters; // Desired distance to move (in meters)
    private double targetPositionMeters; // Final target position for the robot

    private double recalculationThreshold;

    /**
     * DriveDynamic Constructor
     * 
     * @param drivetrain             The swerve drivetrain subsystem
     * @param relativeDistanceMeters The desired distance to move forward (in
     *                               meters)
     */
    public DriveDynamicGradual(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.tag_id = tag_id;

        this.relativeDistanceMeters = visionSubsystem.getTagDistanceAndAngle(3).getDistanceMeters() - 0.1;

        // Initialize the ProfiledPIDController with PID constants and constraints
        // controller = new ProfiledPIDController(Constants.P_ARM_PID_P,
        // Constants.P_ARM_PID_I, Constants.P_ARM_PID_D,
        // new Constraints(0.2, 0.1)); // Velocity and Acceleration constraints

        // controller.setTolerance(0.02, 0.02); // Tolerance for position and velocity
        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        // Get the current robot position in meters
        currentPositionMeters = drivetrain.getState().Pose.getTranslation().getX();

        // Calculate the target position by adding the relative distance to the current
        // position
        targetPositionMeters = currentPositionMeters + relativeDistanceMeters;

        recalculationThreshold = ((relativeDistanceMeters - currentPositionMeters) / 3)
                + currentPositionMeters;

        // // Set the goal in the controller
        // controller.setGoal(targetPositionMeters);

        System.out.println(
                "Moving forward " + relativeDistanceMeters + " meters. Target: " + targetPositionMeters + " meters.");

        System.out.println("Current: " + currentPositionMeters);
        System.out.println("ToRecalc: " + recalculationThreshold);
    }

    @Override
    public void execute() {
        // Get the current robot position in meters
        currentPositionMeters = drivetrain.getState().Pose.getTranslation().getX();

        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.5));

        // System.out.println("C: " + currentPositionMeters);
        // System.out.println("T: " + recalculationThreshold);
        // System.out.println("");

        if (currentPositionMeters <= recalculationThreshold) {
            System.out.println("Distance Reassigned: " + relativeDistanceMeters);
            this.relativeDistanceMeters = visionSubsystem.getTagDistanceAndAngle(3).getDistanceMeters() - 0.1;
            recalculationThreshold = ((relativeDistanceMeters - currentPositionMeters) *
                    (1 / 3)) + currentPositionMeters;
        }
    }

    @Override
    public boolean isFinished() {
        if (currentPositionMeters >= targetPositionMeters) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("Command " + (interrupted ? "interrupted" : "completed") + ". Final robot position: "
                + drivetrain.getState().Pose.getTranslation().getX() + " meters.");
        System.out.println("Target: " + targetPositionMeters + " meters.");
        System.out.println("Relative: " + relativeDistanceMeters + " meters.");
    }

    public double getGoal() {
        return targetPositionMeters;
    }

    public double getRelativeDistance() {
        return relativeDistanceMeters;
    }

}