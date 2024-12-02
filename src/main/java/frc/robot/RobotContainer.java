package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ApproachTagTeleop;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.DistanceAndAngle;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainSubsystem.TunerConstants;
import frc.robot.util.AutonomousChooser;

/**
 * TODO: add rotation control to the Operator Controller
 */

public class RobotContainer {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final ChassisSubsystem m_ChassisSubsystem;
    // private final VisionSubsystem m_VisionSubsystem;
    private final CommandSwerveDrivetrain m_DrivetrainSubsystem = TunerConstants.DriveTrain;

    private final VacuumMaster m_VacuumMaster;
    private final VacummSubystem m_VacummSubystem1;
    private final VacummSubystem m_VacummSubystem2;
    private final VacummSubystem m_VacummSubystem3;

    // private final VacummSubystem m_VacummSubystem;
    private final VisionSubsystem m_VisionSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final ApproachTagTeleop m_ApproachTagTeleop;
    // private final ClimberSubsystem m_ClimberSubsystem;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final AutonomousChooser autonomousChooser = new AutonomousChooser();
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController m_driveController = new CommandXboxController(
            Constants.CONTROLLER_USB_PORT_DRIVER);
    private final CommandXboxController m_operatorController = new CommandXboxController(
            Constants.CONTROLLER_USB_PORT_OPERATOR);

    private boolean slow = false;
    private boolean roll = false;

    /**
     * The robot container. Need I say more?
     */
    public RobotContainer() {
        m_ChassisSubsystem = new ChassisSubsystem();
        m_ArmSubsystem = new ArmSubsystem();

        // m_VacummSubystem = new VacummSubystem();

        m_VacummSubystem1 = new VacummSubystem(Constants.VAC_1_ID);
        m_VacummSubystem2 = new VacummSubystem(Constants.VAC_2_ID);
        m_VacummSubystem3 = new VacummSubystem(Constants.VAC_3_ID);
        m_VacuumMaster = new VacuumMaster(m_VacummSubystem1, m_VacummSubystem2,
                m_VacummSubystem3);

        m_VisionSubsystem = new VisionSubsystem();

        // m_ClimberSubsystem = new ClimberSubsystem();
        registerAutoCommands();
        System.out.println("container created");
        this.m_ApproachTagTeleop = null;
        autoChooser = AutoBuilder.buildAutoChooser();
        configureShuffleBoard();
        resetDrive();
        configureButtonBindings();
    }

    /**
     * Reset the default drive command
     */
    public void resetDrive() {
        m_DrivetrainSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
                m_DrivetrainSubsystem.applyRequest(
                        () -> drive.withVelocityX(square(deadband(m_driveController.getLeftY(), 0.1)) * MaxSpeed) // Drive
                                // forward
                                // with
                                // negative Y (forward)
                                .withVelocityY(square(deadband(m_driveController.getLeftX(), 0.1)) * MaxSpeed) // Drive
                                                                                                               // left
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                // (left)
                                .withRotationalRate(
                                        -square(clampAdd(deadband(m_driveController.getRightX(), 0.1),
                                                deadband(m_operatorController.getLeftX(), 0.1)) * MaxAngularRate)) // Drive
                // counterclockwise
                // with
                // negative X (left)
                ));
        m_DrivetrainSubsystem.seedFieldRelative();
        if (Utils.isSimulation()) {
            m_DrivetrainSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
    }

    /**
     * Get the main controller
     * 
     * @return The main controller
     */
    public CommandXboxController getMainController() {
        return m_driveController;
    }

    /**
     * Register Auto Commands
     */
    public void registerAutoCommands() {
        // NamedCommands.registerCommand("Shoot",
        // new AutoShootCommand(m_ManipulatorSubsystem, m_DrivetrainSubsystem,
        // m_ArmSubsystem));
        // NamedCommands.registerCommand("Intake", new
        // IntakeCommand(m_ManipulatorSubsystem));
        NamedCommands.registerCommand("Reset Pose", new InstantCommand(() -> m_DrivetrainSubsystem.alignToVision()));
        NamedCommands.registerCommand("Slam Arm",
                new FunctionalCommand(() -> m_ArmSubsystem.setGoal(Constants.BACKWARD_SOFT_STOP * 360), () -> {
                }, interrupted -> {
                }, () -> m_ArmSubsystem.atGoal()).andThen(new WaitCommand(0.4)).withTimeout(2));
        NamedCommands.registerCommand("Shoot No Aim", new InstantCommand(() -> {
        }));
    }

    /**
     * Set up the Shuffleboard
     */
    public void configureShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
        tab.addBoolean("SLOW", () -> isSlow()).withPosition(1, 1);
        tab.addBoolean("ROLL", () -> isRoll()).withPosition(2, 1);
        // tab.addBoolean("Note?", () -> m_ManipulatorSubsystem.noteSensor.getRange() <=
        // 150);
        tab.addDouble("X", () -> m_DrivetrainSubsystem.getState().Pose.getX());
        tab.addDouble("Y", () -> m_DrivetrainSubsystem.getState().Pose.getY());
        tab.addDouble("R", () -> m_DrivetrainSubsystem.getState().Pose.getRotation().getDegrees());
        tab.add("Auto Chooser", autoChooser);

        // Vac
        tab.addInteger("Active Vacuum", () -> m_VacuumMaster.getTargetVacAsInt());
        tab.addString("Vac 1 Status", () -> m_VacummSubystem1.getState());
        tab.addString("Vac 2 Status", () -> m_VacummSubystem2.getState());
        tab.addString("Vac 3 Status", () -> m_VacummSubystem3.getState());
    }

    /**
     * Setup all of the button controls for the robot
     */
    public void configureButtonBindings() {

        // DRIVE CONTROLLER

        m_driveController.back().onTrue(m_DrivetrainSubsystem.runOnce(() -> m_DrivetrainSubsystem.seedFieldRelative()));
        m_driveController.leftBumper().onTrue(new InstantCommand(() -> toggleSlow()));
        m_driveController.rightBumper().onTrue(new InstantCommand(() -> toggleRoll()));

        m_driveController.start().onTrue(m_DrivetrainSubsystem.runOnce(() -> m_DrivetrainSubsystem.alignToVision()));

        m_driveController.povUp().whileTrue(m_ArmSubsystem.runOnce(() -> m_ArmSubsystem.raise()));
        m_driveController.povDown().whileTrue(m_ArmSubsystem.runOnce(() -> m_ArmSubsystem.lower()));

        m_driveController.a().onTrue(
                m_VisionSubsystem.runOnce(() -> {
                    System.out.println("started test case");
                    m_VisionSubsystem.doStaticAlign(m_DrivetrainSubsystem,
                            Constants.TEST_TARGET_TAG);

                }));
        m_driveController.start().onTrue(
                m_VisionSubsystem.runOnce(() -> {
                    m_VisionSubsystem.grabMisc(Constants.TEST_TARGET_TAG);
                }));

        // m_driveController.b().onTrue(
        // m_VisionSubsystem.runOnce(() -> {
        // m_VisionSubsystem.approachTeleop(m_DrivetrainSubsystem,
        // Constants.TEST_TARGET_TAG, m_VacummSubystem2, m_ArmSubsystem);
        // }));

        // OPERATOR CONTROLLER

        // m_operatorController.y().onTrue(m_VacummSubystem1.runOnce(() ->
        // m_VacummSubystem1.intakeFull()));
        // m_operatorController.x().onTrue(m_VacummSubystem1.runOnce(() ->
        // m_VacummSubystem1.stop()));

        m_operatorController.y().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.intakeFull()));
        m_operatorController.b().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.intakePartial()));
        m_operatorController.x().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.stop()));
        m_operatorController.a().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.outtake()));

        m_operatorController.povLeft().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.setTargetVac(3)));
        m_operatorController.povUp().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.setTargetVac(2)));
        m_operatorController.povRight().onTrue(m_VacuumMaster.runOnce(() -> m_VacuumMaster.setTargetVac(1)));
        // make one that does all 3 pls :) (on down POV) thx

        m_operatorController.rightBumper().whileTrue(m_ArmSubsystem.runOnce(() -> m_ArmSubsystem.raise()));
        m_operatorController.leftBumper().whileTrue(m_ArmSubsystem.runOnce(() -> m_ArmSubsystem.lower()));

    }

    /**
     * Adjusts the input to remove the tolerance while retaining a smooth line with
     * tolerance as 0 and 100 as 100
     * 
     * @param value     The value to adjust
     * @param tolerance The amount of inner area to remove
     * @return The adjusted value
     */
    public static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    /**
     * Copy sign square
     * 
     * @param value Value to square
     * @return The copy sign square
     */
    public static double square(double value) {
        return Math.copySign(value * value, value);
    }

    public static double clampAdd(double value1, double value2) {
        return Math.max(Math.min(value1 + value2, 1), -1);
    }

    public boolean isSlow() {
        return slow;
    }

    public boolean isRoll() {
        return roll;
    }

    public void toggleSlow() {
        slow = !slow;
        if (slow && roll) {
            roll = false;
        }
        updateSpeeds();
    }

    public void toggleRoll() {
        roll = !roll;
        if (slow && roll) {
            slow = false;
        }
        updateSpeeds();
    }

    private void updateSpeeds() {
        if (slow) {
            MaxSpeed = 1;
            MaxAngularRate = Math.PI * 1;
        } else if (roll) {
            MaxSpeed = 1.5;
            MaxAngularRate = Math.PI * 1; // from 1
        } else {
            MaxSpeed = 3; // from 3
            MaxAngularRate = Math.PI * 1.5;
        }
        System.out.println(MaxSpeed);
    }

    /**
     * Accessor to the Autonomous Chooser
     * 
     * @return The Autonomous Chooser
     */
    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Accessor to the Chassis Subsystem
     * 
     * @return The Chassis Subsystem
     */
    public ChassisSubsystem getChassisSubsystem() {
        return m_ChassisSubsystem;
    }

    /**
     * Accessor to the DriveTrain Subsystem
     * 
     * @return The DriveTrain Subsystem
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return m_DrivetrainSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return m_ArmSubsystem;
    }

    public void alignPigeonVision() {
        m_DrivetrainSubsystem.alignToVision();
    }
}
