package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkFlex intakeMotor;
    private final CANSparkFlex shootMotor;

    public ManipulatorSubsystem(RobotContainer container) {
        intakeMotor = new CANSparkFlex(0, MotorType.kBrushless);
        shootMotor = new CANSparkFlex(1, MotorType.kBrushless);
    }

    public void intake() {

    }

    /**
     * Starts outtaking
     */
    public void shoot() {

    }

    /**
     * Stops taking
     */
    public void stoptake() {
        intakeMotor.stopMotor();
        shootMotor.stopMotor();
    }
}
