package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChassisSubsystem extends SubsystemBase {

    private String serialNumber = "unknown";
    private final NetworkTable visionTable;
    private final NetworkTableEntry visionEntry;
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
    private static ChassisSubsystem instance;
    boolean isBlue = false;

    /**
     * Handles robot wide and generic systems
     */
    public ChassisSubsystem() {
        instance = this;
        serialNumber = RobotController.getSerialNumber();
        System.out.println("SERIALNUMBER=" + serialNumber);
        // SmartDashboard.putData(camera); fix this
        // Get the default instance of NetworkTables
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        // Get the "vision" table from NetworkTables
        visionTable = ntInstance.getTable("Camera");

        // Retrieve handles to specific entries
        visionEntry = visionTable.getEntry("NotePose");
    }

    /**
     * Gets the ChassisSubsystem instance. Makes a new one if not present.
     * 
     * @return The ChassisSubsystem
     */
    public static ChassisSubsystem getChassisInstance() {
        if (instance == null) {
            instance = new ChassisSubsystem();
        }
        return instance;
    }

    public Transform2d getNearestNotePose() {
        double noteDistance = this.visionEntry.getDoubleArray(new double[] { 0.0, 0.0 })[0];
        double noteAngle = this.visionEntry.getDoubleArray(new double[] { 0.0, 0.0 })[1];
        return new Transform2d(new Translation2d(noteDistance, new Rotation2d(noteAngle * (Math.PI / 180.0))),
                new Rotation2d());
    }

    /**
     * Is this swervee?
     * 
     * @return Is this swervee?
     */
    public Boolean isTestRobot() {
        Boolean result = serialNumber.equalsIgnoreCase(Constants.TEST_ROBORIO_SERIAL_NUMBER);
        System.out.println("RIOTEST=" + result);
        return result;
    }

    public void flash() {
        System.out.println("flash here");
    }

    public static ChassisSubsystem getInstance() {
        return instance;
    }

    @Override
    public void periodic() {

    }
}
