package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

/**
 * AlignToSpeakerCommand
 */
public class AlignToSpeakerCommand extends Command {

    CommandSwerveDrivetrain subsystem;
    ProfiledPIDController controller;
    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    public AlignToSpeakerCommand(CommandSwerveDrivetrain subDrivetrain) {
        System.out.println("ALIGN TO THE TING");
        subsystem = subDrivetrain;
        addRequirements(subDrivetrain);
        controller = new ProfiledPIDController(2, 0.05, 0.1, new Constraints(Math.PI * 4, Math.PI * 3));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(0.03, 0.03);
    }

    @Override
    public void initialize() {
        /*
         * swap > if shooting wrong target (also in CommandSwerveDriveTrain.java and a
         * second one in here)
         */
        Translation2d offset = (subsystem.getState().Pose.getX() > 8.4 ? new Translation2d(15.7, 5.55)
                : new Translation2d(0.25, 5.55))
                .minus(subsystem.getState().Pose.getTranslation());
        controller.setGoal(offset.getAngle().plus(new Rotation2d(Math.PI)).getRadians());
        System.out.println(Math.toDegrees(controller.getGoal().position));
    }

    @Override
    public void execute() {
        /*
         * swap > if shooting wrong target (also in CommandSwerveDriveTrain.java and a
         * second one in here)
         */
        Translation2d offset = (subsystem.getState().Pose.getX() > 8.4 ? new Translation2d(16.3, 5.55)
                : new Translation2d(0.3, 5.55))
                .minus(subsystem.getState().Pose.getTranslation());
        double rate = controller.calculate(subsystem.getState().Pose.getRotation().getRadians(),
                offset.getAngle().plus(new Rotation2d(Math.PI)).getRadians());
        if (rate < 0 - controller.getPositionTolerance()) {
            rate = Math.min(rate, Math.abs(controller.getPositionError()) > .1 ? -1.2 : -0.6);
        } else if (rate > 0 + controller.getPositionTolerance()) {
            rate = Math.max(rate, Math.abs(controller.getPositionError()) > .1 ? 1.2 : 0.6);
        }
        // System.out.println(rate);
        subsystem.setControl(request.withRotationalRate(rate));
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted);
        System.out.println(Math.toDegrees(controller.getGoal().position));
        System.out.println(subsystem.getState().Pose.getRotation().getDegrees());
    }
}