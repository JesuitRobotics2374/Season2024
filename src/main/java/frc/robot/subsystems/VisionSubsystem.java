package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable visionTable;
    private final NetworkTableEntry visionEntry;

    public VisionSubsystem() {
        // Get the default instance of NetworkTables
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        // Get the "vision" table from NetworkTables
        visionTable = ntInstance.getTable("Camera");

        // Retrieve handles to specific entries
        visionEntry = visionTable.getEntry("NotePose");
    }

    public Translation2d getNearestNotePose() {
        double noteDistance = this.visionEntry.getDoubleArray(new double[] { 0.0, 0.0 })[0];
        double noteAngle = this.visionEntry.getDoubleArray(new double[] { 0.0, 0.0 })[1];
        return new Translation2d(noteDistance, new Rotation2d(noteAngle*(Math.PI/180.0)));
    }


    @Override
    public void periodic() {
    }
}
