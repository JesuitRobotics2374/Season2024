package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkFlex shooterMotorA;
    private final CANSparkFlex shooterMotorB;
    private final CANSparkFlex intakeMotor;
    private final DigitalInput noteSensor;

    private boolean previousNoteSensorRead;

    private final double shooterSpeed = 0.8;
    private final double intakeSpeed = 0.3;

    public ManipulatorSubsystem(RobotContainer container) {
        shooterMotorA = new CANSparkFlex(Constants.SHOOTER_MOTOR_A_ID, MotorType.kBrushless);
        shooterMotorB = new CANSparkFlex(Constants.SHOOTER_MOTOR_B_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkFlex(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        noteSensor = new DigitalInput(Constants.SENSOR_PORT);
        previousNoteSensorRead = noteSensor.get();
    }

    public void startShooter() {
        shooterMotorA.set(shooterSpeed);
        shooterMotorB.set(shooterSpeed);
    }

    public void stopShooter() {
        shooterMotorA.stopMotor();
        shooterMotorB.stopMotor();
    }

    public void startIntake() {
        intakeMotor.set(intakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void checkNote() {
        if (previousNoteSensorRead != noteSensor.get()) {
            stopIntake();
        }

        previousNoteSensorRead = noteSensor.get();
    }
}
