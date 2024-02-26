package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
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
    private final TimeOfFlight noteSensor;

    boolean intake = false;
    boolean previousSensorRead = false;

    private final double shooterSpeed = 0.7;
    private final double intakeSpeed = 0.3;
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);

    public ManipulatorSubsystem() {
        shooterMotorA = new CANSparkFlex(Constants.SHOOTER_MOTOR_A_ID, MotorType.kBrushless);
        shooterMotorB = new CANSparkFlex(Constants.SHOOTER_MOTOR_B_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        noteSensor = new TimeOfFlight(Constants.SENSOR_PORT);
        tab.addDouble("Shooter Speed", () -> getShooterSpeed());
        tab.addDouble("TOF range", () -> noteSensor.getRange());
    }

    public void intake() {
        if (noteSensor.getRange() > 100) {
            intake = true;
        }
        startIntake();
    }

    public void shoot(double distance) {
        // double angle = calculateAngle(Constants.AIR_DRAG, Constants.DELTA_HEIGHT, shooterSpeedToMPerS(), distance, )
    }

    public void shooterSpeedToMPerS() {
        return ;
    }

    public double calculateAngle(double a, double h, double v, double d, double n, double sqrtVal, double sqrtVal2, double tanVal) {
        if (d != (0.451601 * a * v + 0.451601 * h * v) && v * (d * (sqrtVal - 15795133 * h) + h * v * (7133094 * a + 7133094 * h) + 7133094 * d * d * v) != 0) {
            double atanVal = Math.atan((Math.sqrt(v * v * (50881030012836l * h * h - 50881030012836l * a * a) + 225336336863004l * a * d * v + d * d * (50881030012836l * v * v - 249486226487689l)) + 7133094 * d * v) / (-7133094 * a * v + 15795133 * d - 7133094 * h * v));
            return 2 * (atanVal + 3.14159 * n);
        } else {
            return Double.NaN;
        }
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

    public void reverse() {
        intakeMotor.set(-intakeSpeed);
    }

    @Override
    public void periodic() {
        if (intake && (noteSensor.getRange() <= 100)) {
            stopIntake();
            intake = false;
        }
    }

    public double getShooterSpeed() {
        return shooterMotorA.getEncoder().getVelocity() * Units.inchesToMeters(2 * Math.PI) / 60;
    }

    public boolean shooterAtMaxSpeed() {
        return getShooterSpeed() > 11.6;
    }
}
