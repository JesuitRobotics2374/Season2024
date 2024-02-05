package frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.Splines.SplineAbstract;

public class HolonomicPathBuilder {
    LinkedList<SplineAbstract> pathList = new LinkedList<>();

    public HolonomicPathBuilder() {

    }

    public HolonomicPathBuilder andThen(SplineAbstract spline) {
        pathList.add(spline);
        return this;
    }

    public void clense(Pose2d currentPose) {
        if (!pathList.isEmpty() && pathList.peekFirst().atGoal()) {
            pathList.removeFirst();
            if (!pathList.isEmpty()) {
                pathList.peekFirst().initialize(currentPose);
            }
        }
    }

    public boolean isFinished() {
        return pathList.isEmpty();
    }
}
