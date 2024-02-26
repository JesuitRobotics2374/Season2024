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
        Xcontroller = new ProfiledPIDController(1.1, 0.15, 0.15, XYconstraints);
        Ycontroller = new ProfiledPIDController(1.1, 0.15, 0.15, XYconstraints);
        Rcontroller = new ProfiledPIDController(0.4, 0.06, 0.04, Rconstraints);
        Rcontroller.enableContinuousInput(0, Math.PI * 2);
        Xcontroller.setGoal(endPose2d.getX());
        Ycontroller.setGoal(endPose2d.getY());
        Rcontroller.setGoal(endPose2d.getRotation().getDegrees());
        Xcontroller.setTolerance(0.05, 0.5);
        Ycontroller.setTolerance(0.05, 0.5);
        Rcontroller.setTolerance(0.02);
        this.fieldCentric = fieldCentric;
    }

    public Line(Constraints XYconstraints, Constraints Rconstraints, Pose2d endPose2d, boolean fieldCentric,
            double positionTolerance, double velocityTolerance) {
        this.endPose2d = endPose2d;
        Xcontroller = new ProfiledPIDController(1.1, 0.15, 0.15, XYconstraints);
        Ycontroller = new ProfiledPIDController(1.1, 0.15, 0.15, XYconstraints);
        Rcontroller = new ProfiledPIDController(0.4, 0.06, 0.04, Rconstraints);
        Rcontroller.enableContinuousInput(0, Math.PI * 2);
        Xcontroller.setGoal(endPose2d.getX());
        Ycontroller.setGoal(endPose2d.getY());
        Rcontroller.setGoal(endPose2d.getRotation().getRadians());
        Xcontroller.setTolerance(positionTolerance, velocityTolerance);
        Ycontroller.setTolerance(positionTolerance, velocityTolerance);
        Rcontroller.setTolerance(0.02);
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void initialize(Pose2d currentPose2d) {
        if (!fieldCentric) {
            endPose2d = new Pose2d(endPose2d.getTranslation().plus(currentPose2d.getTranslation()),
                    endPose2d.getRotation().plus(currentPose2d.getRotation()));
        }
        Xcontroller.setGoal(endPose2d.getX());
        Ycontroller.setGoal(endPose2d.getY());
        Rcontroller.setGoal(endPose2d.getRotation().getRadians());
        System.out.println(currentPose2d);
        System.out.println(endPose2d);
        Xcontroller.reset(currentPose2d.getX());
        Ycontroller.reset(currentPose2d.getY());
        Rcontroller.reset(currentPose2d.getRotation().getRadians());
    }

    @Override
    public ChassisSpeeds getMovement(Pose2d currentPose2d) {
        ChassisSpeeds speeds = new ChassisSpeeds(Xcontroller.calculate(currentPose2d.getX()),
                Ycontroller.calculate(currentPose2d.getY()),
                Rcontroller.calculate(currentPose2d.getRotation().getRadians()));
        if (Math.abs(speeds.vxMetersPerSecond) > Xcontroller.getConstraints().maxVelocity) {
            speeds.vxMetersPerSecond = Math.copySign(Xcontroller.getConstraints().maxVelocity,
                    speeds.vxMetersPerSecond);
        }
        if (Math.abs(speeds.vyMetersPerSecond) > Ycontroller.getConstraints().maxVelocity) {
            speeds.vyMetersPerSecond = Math.copySign(Ycontroller.getConstraints().maxVelocity,
                    speeds.vyMetersPerSecond);
        }
        if ((currentPose2d.getRotation().minus(endPose2d.getRotation()).getDegrees() + 360) % 360 > 4
                && (currentPose2d.getRotation().minus(endPose2d.getRotation()).getDegrees() + 360) % 360 < 100) {
            speeds.omegaRadiansPerSecond -= Math.PI / 3;
            // System.out.println("a");
        } else if ((currentPose2d.getRotation().minus(endPose2d.getRotation()).getDegrees() + 360) % 360 < 356
                && (currentPose2d.getRotation().minus(endPose2d.getRotation()).getDegrees() + 360) % 360 > 260) {
            speeds.omegaRadiansPerSecond += Math.PI / 3;
            // System.out.println("b");
        }
        // System.out.println((currentPose2d.getRotation().minus(endPose2d.getRotation()).getDegrees()
        // + 360) % 360);
        if (Math.abs(speeds.omegaRadiansPerSecond) > Rcontroller.getConstraints().maxVelocity) {
            speeds.omegaRadiansPerSecond = Math.copySign(Rcontroller.getConstraints().maxVelocity,
                    speeds.omegaRadiansPerSecond);
        }
        return speeds;
    }

    @Override
    public boolean atGoal() {
        return Xcontroller.atGoal() && Ycontroller.atGoal() && Rcontroller.atGoal();
    }
}
