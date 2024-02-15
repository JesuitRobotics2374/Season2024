package frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class FollowCommand extends Command {

    HolonomicPathBuilder builder;
    CommandSwerveDrivetrain subsystem;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(0.05).withRotationalDeadband(0.05); // I
                                                                                                                     // want
                                                                                                                     // field-centric
    // driving in open loop

    public FollowCommand(CommandSwerveDrivetrain subsystem, HolonomicPathBuilder builder) {
        this.builder = builder;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        builder.pathList.peekFirst().initialize(subsystem.getState().Pose);
        // subsystem.seedFieldRelative(new
        // Pose2d(subsystem.getState().Pose.getTranslation(), new Rotation2d()));

    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = builder.pathList.peekFirst().getMovement(subsystem.getState().Pose);
        System.out.println("Speeds: " + speeds);
        // System.out.println("Pose: " + subsystem.getState().Pose);
        subsystem.setControl(drive.withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
        builder.clense(subsystem.getState().Pose);

    }

    @Override
    public boolean isFinished() {
        return builder.isFinished();
    }
}
