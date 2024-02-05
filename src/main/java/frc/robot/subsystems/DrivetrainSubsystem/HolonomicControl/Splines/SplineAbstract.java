package frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.Splines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SplineAbstract {

    public abstract ChassisSpeeds getMovement(Pose2d currentPose2d);

    public abstract boolean atGoal();

    public abstract void initialize(Pose2d currentPose2d);
}
