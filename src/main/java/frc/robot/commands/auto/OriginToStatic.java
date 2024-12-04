package frc.robot.commands.auto;

import java.util.Arrays;

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
public class OriginToStatic extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int tag_id;
    // private ProfiledPIDController controller;
    private double currentPositionMeters;
    private double relativeDistanceMeters; // Desired distance to move (in meters)
    private double targetPositionMeters; // Final target position for the robot

    private double static_x;
    private double static_y;
    private double static_r;

    private boolean doneMoving;
    private boolean doneRotating;

    private int debugCount = 0;

    // Method to check if an array contains a specific integer
    private static boolean contains(int[] array, int target) {
        // Iterate through each element in the array
        for (int num : array) {
            // If the current element equals the target, return true
            if (num == target) {
                return true;
            }
        }
        // If the target is not found, return false
        return false;
    }

    /**
     * DriveDynamic Constructor
     * 
     * @param drivetrain             The swerve drivetrain subsystem
     * @param relativeDistanceMeters The desired distance to move forward (in
     *                               meters)
     */
    public OriginToStatic(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.tag_id = tag_id;

        if (contains(Constants.A_GROUP_MEMBERS, tag_id)) {
            this.static_x = Constants.A_GROUP_X;
            this.static_y = Constants.A_GROUP_Y;
            this.static_r = Constants.ALL_GROUPS_ROTATION;
        } else if (contains(Constants.B_GROUP_MEMBERS, tag_id)) {
            this.static_x = Constants.B_GROUP_X;
            this.static_y = Constants.B_GROUP_Y;
            this.static_r = Constants.ALL_GROUPS_ROTATION;
        } else if (contains(Constants.C_GROUP_MEMBERS, tag_id)) {
            this.static_x = Constants.C_GROUP_X;
            this.static_y = Constants.C_GROUP_Y;
            this.static_r = Constants.ALL_GROUPS_ROTATION + 180.0;
        } else if (contains(Constants.D_GROUP_MEMBERS, tag_id)) {
            this.static_x = Constants.D_GROUP_X;
            this.static_y = Constants.D_GROUP_Y;
            this.static_r = Constants.ALL_GROUPS_ROTATION + 180.0;
        } else {
            throw new IllegalArgumentException("Invalid tag_id: " + tag_id);
        }
        System.out.println("x: " + static_x);
        System.out.println("y: " + static_y);
        System.out.println("r: " + static_r);

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
        doneMoving = false;
        doneRotating = false;
    }

    @Override
    public void execute() {

        // debugCount++;

        // if (debugCount < 60) {
        // return;
        // }

        // debugCount = 0;

        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        double currentRotation = drivetrain.getState().Pose.getRotation().getDegrees();
        double distanceToTarget = robotPosition.getDistance(new Translation2d(static_x, static_y));

        // System.out.println("Goal: X " + static_x + " Y " + static_y);
        // System.out.println(" Now: X " + robotPosition.getX() + " Y " +
        // robotPosition.getY());
        System.out.println(
                "Rotation: " + currentRotation + " Goal: " + static_r + " Diff: " + (currentRotation - static_r));
        System.out.println(doneMoving + " " + doneRotating);

        // TODO: Check for rotation and translation completenes separately

        if (distanceToTarget < 0.3) {
            doneMoving = true;
            System.out.println("Done moving.");
        }
        if (Math.abs(currentRotation - static_r) < 5.0) {
            doneRotating = true;
            System.out.println("Done rotating.");
        }

        double velocityX = 0;
        double velocityY = 0;
        double rotationalRate = 0;

        if (!doneMoving) {
            // Calculate the direction to the target
            double deltaX = static_x - robotPosition.getX();
            double deltaY = static_y - robotPosition.getY();

            // Normalize the velocities
            double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            velocityX = (deltaX / magnitude) * 0.5; // Scale to desired speed
            velocityY = (deltaY / magnitude) * 0.5; // Scale to desired speed

            // System.out.println(" VEL: X " + velocityX + " Y " + velocityY);
        }

        if (!doneRotating) {
            // Calculate the rotational rate
            double rotationError = static_r - currentRotation;
            rotationalRate = rotationError * 0.025; // Scale to desired rotational speed
        }

        System.out.println("RR: " + rotationalRate);

        if (!(doneMoving && doneRotating)) {
            drivetrain.setControl(
                    new SwerveRequest.FieldCentric().withVelocityX(velocityY).withVelocityY(-velocityX)
                            .withRotationalRate(-rotationalRate));
        }

    }

    @Override
    public boolean isFinished() {
        return doneMoving && doneRotating;
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