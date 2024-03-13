package frc.robot.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
    Field2d field = new Field2d();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pose = table.getEntry("botpose");
    ArrayList<Double> xList = new ArrayList<>();
    ArrayList<Double> yList = new ArrayList<>();
    ArrayList<Double> rList = new ArrayList<>();
    ArrayList<Double> tList = new ArrayList<>();
    static CommandSwerveDrivetrain instance;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        tab.add(field).withPosition(0, 3).withSize(5, 3);
        instance = this;
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        tab.add(field).withPosition(0, 3).withSize(5, 3);
        tab.addDouble("Offset", () -> this.m_fieldRelativeOffset.getDegrees());
        tab.addDouble("Pigeon", () -> (getState().Pose.getRotation().getDegrees()));
        Matrix<N3, N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        matrix.set(0, 0, 4);
        matrix.set(1, 0, 4);
        matrix.set(2, 0, .9);
        setVisionMeasurementStdDevs(matrix);
        instance = this;
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(0.9, 0, .5),
                        new PIDConstants(1, 0, 0.01),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {

                    return false;
                },
                this); // Subsystem for requirements

    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        double[] array = pose.getDoubleArray(new double[0]);
        boolean flag = isTargetValid(array);
        // System.out.println(array.length);
        if (array.length > 0) {
            Pose2d visionPose2d = new Pose2d(array[0] + 8.23, array[1] + 4.1, new Rotation2d(Math.toRadians(array[5])));
            field.getObject("Vision").setPose(
                    new Pose2d(visionPose2d.getX() + .77, visionPose2d.getY() + .1, visionPose2d.getRotation()));// 9,
            // 4.2
            // new Pose2d(array[0] + 8.23, array[1] + 4.1, new
            // Rotation2d(Math.toRadians(array[5]))));
            if (visionPose2d.getX() != 0 || visionPose2d.getY() != 0) {
                double offset = Math
                        .sqrt(Math.pow(visionPose2d.relativeTo(getState().Pose).getX(), 2)
                                + Math.pow(visionPose2d.relativeTo(getState().Pose).getY(), 2));
                if (flag && offset < 1.5 && Math.abs(visionPose2d.getRotation().getDegrees()
                        - getState().Pose.getRotation().getDegrees()) < 30) {
                    addVisionMeasurement(visionPose2d,
                            Timer.getFPGATimestamp() - (array[6] / 1000.0));
                } // Timer.getFPGATimestamp() - (botpose[6]/1000.0)
            }
        }
        field.setRobotPose(
                new Pose2d(getState().Pose.getX() + .77, getState().Pose.getY() + .1, getState().Pose.getRotation()));
    }

    public boolean isTargetValid(double[] array) {
        if (array.length < 3) {
            return false;
        }
        xList.add(array[0]);
        yList.add(array[1]);
        rList.add(array[5]);
        tList.add(array[6]);
        if (xList.size() < 10) {
            return false;
        }
        double[] x = new double[xList.size() - 1];
        double[] y = new double[yList.size() - 1];
        double[] r = new double[rList.size() - 1];

        for (int i = 0; i < x.length; i++) {
            x[i] = (xList.get(i) - xList.get(i + 1)) / (tList.get(i + 1) / 1000);
            y[i] = (yList.get(i) - yList.get(i + 1)) / (tList.get(i + 1) / 1000);
            r[i] = (rList.get(i) - rList.get(i + 1)) / (tList.get(i + 1) / 1000);
        }
        for (int i = 0; i < x.length - 1; i++) {
            if (Math.abs(x[i] - x[i + 1]) > 0.9 || Math.abs(y[i] - y[i + 1]) > 0.9 || Math.abs(x[i] - x[i + 1]) > 90) {
                xList.remove(0);
                yList.remove(0);
                rList.remove(0);
                tList.remove(0);
                return false;
            }
        }

        // System.out.println(x[0] - x[1]);
        xList.remove(0);
        yList.remove(0);
        rList.remove(0);
        tList.remove(0);
        return true;
    }

    public void alignToVision() {
        System.out.println(field.getObject("Vision").getPose());
        // getState().Pose = field.getObject("Vision").getPose();
        seedFieldRelative(new Pose2d(field.getObject("Vision").getPose().getTranslation().getX() - 0.77,
                field.getObject("Vision").getPose().getTranslation().getY() - 0.1, new Rotation2d()));
        seedFieldRelative();
        seedFieldRelative(new Pose2d(field.getObject("Vision").getPose().getTranslation().getX() - 0.77,
                field.getObject("Vision").getPose().getTranslation().getY() - 0.1,
                field.getObject("Vision").getPose().getRotation()));

        alignWithAliance();
    }

    public void alignWithAliance() {
        boolean flag = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            flag = alliance.get() == Alliance.Red;
            m_fieldRelativeOffset = (flag ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        }
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void setStates(SwerveModuleState[] states) {
        getState().ModuleTargets = states;
    }

    public static CommandSwerveDrivetrain getInstance() {
        return instance;
    }

    public double getDistanceToSpeaker() {
        /* swap > if wrong target (also in AlignToSpeakerComand.java) */
        double offset = (getState().Pose.getX() > 8.4
                ? new Translation2d(Constants.RED_SPEAKER_X, Constants.RED_SPEAKER_Y)
                : new Translation2d(0, Constants.RED_SPEAKER_Y))
                .getDistance(getState().Pose.getTranslation());
        return offset;
    }

    public boolean goToNote() {
        Transform2d notePose = ChassisSubsystem.getInstance().getNearestNotePose();
        if (notePose == null || notePose.getTranslation().getNorm() > 5) {
            return false;
        }
        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                getState().Pose,
                new Pose2d(getState().Pose.transformBy(notePose).getTranslation(), getState().Pose.getRotation()
                        .plus(Rotation2d.fromRadians(Math.tan(notePose.getY() / notePose.getX())))));

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a
                                                                         // differential drivetrain, the angular
                                                                         // constraints have no effect.
                new GoalEndState(0.0, getState().Pose.getRotation()
                        .plus(Rotation2d.fromRadians(Math.tan(notePose.getY() / notePose.getX())))) // Goal end state.
                                                                                                    // You can set a
                                                                                                    // holonomic
                                                                                                    // rotation
        // here. If using a differential drivetrain, the
        // rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        Command command = AutoBuilder.followPath(path).raceWith(new IntakeCommand(ManipulatorSubsystem.getInstance()));
        command.addRequirements(this);
        command.schedule();
        return true;
    }
}
