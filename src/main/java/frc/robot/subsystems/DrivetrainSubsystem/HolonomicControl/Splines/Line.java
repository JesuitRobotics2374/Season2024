package frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.Splines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Line extends SplineAbstract {
    Pose2d endPose2d;
    ProfiledPIDController Xcontroller;
    ProfiledPIDController Ycontroller;
    ProfiledPIDController Rcontroller;
    boolean fieldCentric;

    public Line(Constraints XYconstraints, Constraints Rconstraints, Pose2d endPose2d, boolean fieldCentric) {
        this.endPose2d = endPose2d;
        Xcontroller = new ProfiledPIDController(1.3, 0.3, 0.1, XYconstraints);
        Ycontroller = new ProfiledPIDController(1.3, 0.3, 0.1, XYconstraints);
        Rcontroller = new ProfiledPIDController(0.4, 0.08, 0.025, Rconstraints);
        Rcontroller.enableContinuousInput(0, Math.PI * 2);
        Xcontroller.setGoal(endPose2d.getX());
        Ycontroller.setGoal(endPose2d.getY());
        Rcontroller.setGoal(endPose2d.getRotation().getDegrees());
        Rcontroller.setTolerance(Math.PI / 36);
        Xcontroller.setTolerance(0.05, 0.5);
        Ycontroller.setTolerance(0.05, 0.5);
        this.fieldCentric = fieldCentric;
    }

    public Line(Constraints XYconstraints, Constraints Rconstraints, Pose2d endPose2d, boolean fieldCentric,
            double positionTolerance, double velocityTolerance) {
        this.endPose2d = endPose2d;
        Xcontroller = new ProfiledPIDController(1.3, 0.3, 0.1, XYconstraints);
        Ycontroller = new ProfiledPIDController(1.3, 0.3, 0.1, XYconstraints);
        Rcontroller = new ProfiledPIDController(0.4, 0.08, 0.025, Rconstraints);
        Rcontroller.enableContinuousInput(0, Math.PI * 2);
        Xcontroller.setGoal(endPose2d.getX());
        Ycontroller.setGoal(endPose2d.getY());
        Rcontroller.setGoal(endPose2d.getRotation().getDegrees());
        Rcontroller.setTolerance(Math.PI / 36);
        Xcontroller.setTolerance(positionTolerance, velocityTolerance);
        Ycontroller.setTolerance(positionTolerance, velocityTolerance);
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void initialize(Pose2d currentPose2d) {
        if (fieldCentric) {
            Xcontroller.setGoal(endPose2d.getX());
            Ycontroller.setGoal(endPose2d.getY());
            Rcontroller.setGoal(endPose2d.getRotation().getDegrees());
        } else {
            Xcontroller.setGoal(endPose2d.getX() + currentPose2d.getX());
            Ycontroller.setGoal(endPose2d.getY() + currentPose2d.getY());
            Rcontroller.setGoal(endPose2d.getRotation().getRadians() + currentPose2d.getRotation().getRadians());
        }
        Xcontroller.reset(currentPose2d.getX());
        Ycontroller.reset(currentPose2d.getY());
        Rcontroller.reset(currentPose2d.getRotation().getRadians());
    }

    @Override
    public ChassisSpeeds getMovement(Pose2d currentPose2d) {
        return new ChassisSpeeds(Xcontroller.calculate(currentPose2d.getX()),
                Ycontroller.calculate(currentPose2d.getY()),
                Rcontroller.calculate(currentPose2d.getRotation().getRadians()));
    }

    @Override
    public boolean atGoal() {
        return Xcontroller.atGoal() && Ycontroller.atGoal() && Rcontroller.atGoal();
    }
}
