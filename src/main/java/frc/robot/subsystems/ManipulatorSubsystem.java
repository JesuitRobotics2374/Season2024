package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final ArmSubsystem m_ArmSubsystem;

    private final CANSparkFlex shooterMotorA;
    private final CANSparkFlex shooterMotorB;
    private final CANSparkFlex intakeMotor;
    private final TimeOfFlight noteSensor;

    boolean shoot = false;
    boolean previousSensorRead = false;

    private final double shooterSpeed = 0.8;
    private final double intakeSpeed = 0.3;

    public ManipulatorSubsystem() {
        m_ArmSubsystem = new ArmSubsystem();

        shooterMotorA = new CANSparkFlex(Constants.SHOOTER_MOTOR_A_ID, MotorType.kBrushless);
        shooterMotorB = new CANSparkFlex(Constants.SHOOTER_MOTOR_B_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkFlex(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        noteSensor = new TimeOfFlight(Constants.SENSOR_PORT);
    }

    public void shoot(double distance) {
        double fireAngle = m_ArmSubsystem.angleCalculator(distance);
        m_ArmSubsystem.setGoal(fireAngle);
        shoot = true;
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

    @Override
    public void periodic() {
        if (m_ArmSubsystem.atGoal() && shoot) {
            startShooter();
            shoot = false;
        }

        if (noteSensor.getRange() >= 10.0) {
            stopShooter();
        }

        if ((noteSensor.getRange() <= 10.0) != previousSensorRead) {
            stopIntake();
        }

        previousSensorRead = noteSensor.getRange() <= 10.0;
    }
}
