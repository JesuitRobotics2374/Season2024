package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import frc.robot.Constants;

/**
 * AlignDynamic - Aligns the robot by a specified robot-relative angle.
 */
public class AlignDynamic extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    // private final ProfiledPIDController controller;
    private final double relativeAngleRadians; // Desired angle to turn (in radians)
    private double targetAngleRadians; // Final target angle for the robot
    private double currentAngleRadians;
    private int signum;

    /**
     * AlignDynamic Constructor
     * 
     * @param drivetrain           The swerve drivetrain subsystem
     * @param relativeAngleDegrees The desired robot-relative rotation (in degrees)
     */
    public AlignDynamic(CommandSwerveDrivetrain drivetrain, double relativeAngleDegrees) {
        this.drivetrain = drivetrain;
        this.relativeAngleRadians = Math.toRadians(relativeAngleDegrees); // Convert to radians

        // Initialize the ProfiledPIDController with PID constants and constraints
        // controller = new ProfiledPIDController(Constants.P_ARM_PID_P,
        // Constants.P_ARM_PID_I, Constants.P_ARM_PID_D,
        // new Constraints(Math.PI * 0.5, Math.PI * 0.3)); // Loosened Velocity and
        // Acceleration constraints

        // controller.enableContinuousInput(-Math.PI, Math.PI); // Handle full circle
        // rotation
        // controller.setTolerance(0.1, 0.5); // Loosened tolerance
        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        // Get the current robot rotation in radians
        currentAngleRadians = -drivetrain.getState().Pose.getRotation().getRadians();

        // Calculate the target angle by adding the relative angle to the current
        // heading
        targetAngleRadians = currentAngleRadians + relativeAngleRadians; // Corrected to add the angle

        // Set the goal in the controller
        // controller.setGoal(targetAngleRadians);

        System.out.println("Aligning to " + Math.toDegrees(relativeAngleRadians) + " degrees. Target: "
                + Math.toDegrees(targetAngleRadians));
    }

    @Override
    public void execute() {
        // Get the current robot angle in radians
        currentAngleRadians = -drivetrain.getState().Pose.getRotation().getRadians();

        // Calculate the necessary rate (turning speed) to reach the target angle
        // double rotationalRate = controller.calculate(currentAngleRadians);

        // Apply the rotational rate to the drivetrain
        signum = (int) (targetAngleRadians / Math.abs(targetAngleRadians));
        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(Math.PI / 3 * -signum));
    }

    @Override
    public boolean isFinished() {
        System.out.println("C: " + currentAngleRadians * (180 / Math.PI));
        System.out.println("T: " + targetAngleRadians * (180 / Math.PI));
        if (signum == -1) {
            if (currentAngleRadians <= targetAngleRadians) {
                return true;
            }
            return false;
        } else {
            if (currentAngleRadians >= targetAngleRadians) {
                return true;
            }
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("we are done! :DDD");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("Command " + (interrupted ? "interrupted" : "completed") + ". Final robot heading: "
                + currentAngleRadians * (180 / Math.PI) + " degrees.");
        System.out.println("Target: " + targetAngleRadians * (180 / Math.PI));
        System.out.println("Relative: " + relativeAngleRadians * (180 / Math.PI));
    }
}
