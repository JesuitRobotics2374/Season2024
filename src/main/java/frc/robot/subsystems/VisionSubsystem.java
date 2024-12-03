// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.ApproachTagAuto;

import frc.robot.commands.ApproachTagTeleop;
import frc.robot.commands.auto.DriveAndSeek;
import frc.robot.commands.auto.DriveDynamic;
import frc.robot.commands.auto.DriveDynamicY;
import frc.robot.commands.auto.OriginToStatic;

import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {

    private static VisionSubsystem instance;
    // ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);

    private int offset = 5000;

    DriveDynamic driveDynamic;

    /** Creates a new VisionSubsystem. */
    public VisionSubsystem() {

        instance = this;

        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");


    }

    public class DistanceAndAngle {
        private final double distance;
        private final double theta;

        public DistanceAndAngle(double distance, double theta) {

            this.distance = distance;
            this.theta = theta;
        }

        public double getDistance() {
            return distance;
        }

        public double getTheta() {
            return theta;
        }

        public double getDistanceMeters() {
            return distance / 39.37;
        }

        @Override
        public String toString() {
            return String.format("Distance: %.2f inches, Angle: %.2f degrees", distance, theta);
        }
    }

    public boolean canSeeTag(int tag_id) {
        int detectedTagId = (int) LimelightHelpers.getFiducialID("");
        return (detectedTagId == tag_id);
    }

    public DistanceAndAngle getTagDistanceAndAngle(int tag_id) {

        int detectedTagId = (int) LimelightHelpers.getFiducialID("");
        if (detectedTagId == tag_id) {

            double tagHeight = LimelightHelpers.getT2DArray("")[15];
            double tagWidth = LimelightHelpers.getT2DArray("")[14];

            double tx = LimelightHelpers.getTX(""); // Horizontal angle offset
            System.out.println("TX: " + tx);
            // double xRot = Math.acos(tagWidth / tagHeight);
            // System.out.println("calc xrot: " + xRot * (180 / Math.PI));

            // double ta = LimelightHelpers.getTA(""); // Tag screen coverage

            double distance = offset / tagHeight; // inches

            return new DistanceAndAngle(distance, tx);
        }

        return new DistanceAndAngle(-1.0, -1.0);
    }

    public Pose3d getTagPose3d(int tag_id) {
        int detectedTagId = (int) LimelightHelpers.getFiducialID("");
        if (detectedTagId == tag_id) {
            return LimelightHelpers.getTargetPose3d_CameraSpace("");
        }
        return null;
    }

    public void raiseOffset() {
        offset += 5;
        System.out.println(offset);
    }

    public void lowerOffset() {
        offset -= 5;
        System.out.println(offset);
    }

    public void approachDynamically(CommandSwerveDrivetrain ds, int tag_id, VacummSubystem vac, ArmSubsystem arm) {
        DistanceAndAngle d = getTagDistanceAndAngle(tag_id);
        // if (d.getDistance() != -1.0 && d.getTheta() != -1.0) {
        // System.out.println("THETA: " + d.getTheta());
        // AlignDynamic align = new AlignDynamic(ds, d.getTheta());
        // ApproachTag approach = new ApproachTag(ds, this, tag_id);
        // }
        ApproachTagAuto a = new ApproachTagAuto(ds, instance, tag_id, vac, arm);
        a.schedule();
    }

    public void approachTeleop(CommandSwerveDrivetrain m_DrivetrainSubsystem,
            int testTargetTag, VacummSubystem m_VacummSubystem, ArmSubsystem m_ArmSubsystem) {
        ApproachTagTeleop a = new ApproachTagTeleop(m_DrivetrainSubsystem, instance, testTargetTag, m_VacummSubystem,
                m_ArmSubsystem);
        a.schedule();
    }


    public void driveDynamically(CommandSwerveDrivetrain ds, int tag_id) {
        DistanceAndAngle d = getTagDistanceAndAngle(tag_id);
        if (d.getDistance() != -1.0 && d.getTheta() != -1.0 && d != null) {
            DriveDynamic drive = new DriveDynamic(ds, this, tag_id, 1.4, 0);
            drive.schedule();
        }
    }

    public void doStaticAlign(CommandSwerveDrivetrain ds, int tag_id) {
        // DistanceAndAngle d = getTagDistanceAndAngle(tag_id);
        OriginToStatic drive = new OriginToStatic(ds, this, tag_id);
        drive.schedule();
    }


    public void panDynamically(CommandSwerveDrivetrain ds, int tag_id) {
        DistanceAndAngle d = getTagDistanceAndAngle(tag_id);
        if (d.getDistance() != -1.0 && d.getTheta() != -1.0 && d != null) {
            DriveDynamicY drive = new DriveDynamicY(ds, this, tag_id, 1, 3);
            drive.schedule();
        }
    }

    public void grabMisc(int tag_id) {
        int detectedTagId = (int) LimelightHelpers.getFiducialID("");
        if (detectedTagId == tag_id) {

            double tagHeight = LimelightHelpers.getT2DArray("")[15];
            double tagWidth = LimelightHelpers.getT2DArray("")[14];
            DistanceAndAngle d = getTagDistanceAndAngle(tag_id);

            double f = (Math.PI / 2) - d.getTheta() - Math.acos(tagWidth / tagHeight);

            System.out.println("resultant: " + f);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void driveStatic(CommandSwerveDrivetrain m_DrivetrainSubsystem, int testTargetTag) {
        // DriveDynamic drive = new DriveDynamic(m_DrivetrainSubsystem, 0.2);
        // driveDynamic = drive;
        // drive.schedule();
    }

    public DriveDynamic getDriveDynamic() {
        return driveDynamic;
    }
}