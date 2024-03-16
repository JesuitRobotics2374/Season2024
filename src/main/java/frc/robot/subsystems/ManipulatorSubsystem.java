package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkFlex shooterMotorA;
    private final CANSparkFlex shooterMotorB;
    private final CANSparkMax intakeMotor;
    public final TimeOfFlight noteSensor;

    boolean intake = false;
    boolean previousSensorRead = false;

    private final double shooterSpeed = -1;
    private final double intakeSpeed = 0.8;
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
    static ManipulatorSubsystem instance;

    public ManipulatorSubsystem() {
        shooterMotorA = new CANSparkFlex(Constants.SHOOTER_MOTOR_A_ID, MotorType.kBrushless);
        shooterMotorB = new CANSparkFlex(Constants.SHOOTER_MOTOR_B_ID, MotorType.kBrushless);
        shooterMotorA.setInverted(false);
        shooterMotorA.setCANTimeout(80);
        shooterMotorB.setInverted(false);
        shooterMotorA.setIdleMode(IdleMode.kCoast);
        shooterMotorB.setIdleMode(IdleMode.kCoast);
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        noteSensor = new TimeOfFlight(Constants.SENSOR_PORT);
        tab.addDouble("Shooter Speed", () -> getShooterSpeed());
        tab.addDouble("TOF range", () -> noteSensor.getRange());
        noteSensor.setRangingMode(RangingMode.Short, 24);
        noteSensor.setRangeOfInterest(9, 9, 11, 11);
        instance = this;
    }

    public void intake() {
        if (noteSensor.getRange() > 150) {
            intake = true;
        }
        startIntake();
    }

    public void startShooter() {
        shooterMotorA.set(shooterSpeed);
        shooterMotorB.set(-shooterSpeed);
    }

    public void stopShooter() {
        shooterMotorA.stopMotor();
        shooterMotorB.stopMotor();
    }

    public void startIntake() {
        intakeMotor.set(intake ? intakeSpeed : 1);
    }

    public void reverse() {
        intakeMotor.set(-intakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void slowClimb() {
        shooterMotorB.set(-0.2);
    }

    @Override
    public void periodic() {
        if (intake && (noteSensor.getRange() <= 150)) {
            stopIntake();
            ChassisSubsystem.getInstance().flash();
            intake = false;
        }
    }

    public double getShooterSpeed() {
        return -shooterMotorA.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42).getVelocity()
                * Units.inchesToMeters(2 * Math.PI) / 60;
    }

    public boolean shooterAtMaxSpeed() {
        return getShooterSpeed() > 13;
    }

    public static ManipulatorSubsystem getInstance() {
        return instance;
    }

    public boolean getIntake() {
        return intake;
    }
}
