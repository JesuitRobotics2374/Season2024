package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    ProfiledPIDController armController = new ProfiledPIDController(.25, 0, 0.05, new Constraints(.8, 1));
    ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
    TalonFX leftMotor = new TalonFX(Constants.LEFT_ARM_MOTOR_ID);
    TalonFX rightMotor = new TalonFX(Constants.RIGHT_ARM_MOTOR_ID);
    CANcoder encoder = new CANcoder(Constants.ARM_ENCODER_ID, Constants.CAN_BUS_NAME_CANIVORE);
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);

    public ArmSubsystem() {
        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.setControl(new Follower(Constants.LEFT_ARM_MOTOR_ID, true));
        encoder.getConfigurator()
                .apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        armController.enableContinuousInput(0, 1);
        armController.setTolerance(0.01, 0.01);
        armController.setGoal(encoder.getAbsolutePosition().getValueAsDouble());
        tab.addDouble("Setpoint", () -> armController.getSetpoint().position);
        tab.addDouble("Point", () -> encoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        // leftMotor.set(
        // armController.calculate(encoder.getAbsolutePosition().getValueAsDouble()));
    }

    public void setGoal(double degrees) {
        armController.setGoal(degrees / 360);
    }

    public boolean atGoal() {
        return armController.atGoal();
    }

    public void raise() {
        armController.setGoal(armController.getGoal().position + .01);
    }

    public void lower() {
        leftMotor.set(armController.getGoal().position - .01);
    }

    public double angleCalculator(double distance) {
        double top = Math.PI / 2;
        double bottom = 0.0;

        for (int cycle = 0; cycle < 100; cycle++) {
            double angle = (top + bottom) / 2;

            double t = distance / Constants.launchVelocity * Math.cos(angle);
            double y = distance * Constants.launchVelocity * Math.sin(angle) - 4.903325 * t * t;

            if (y > Constants.deltaHeight) {
                top = angle;
            } else if (y < Constants.deltaHeight) {
                bottom = angle;
            } else {
                break;
            }
        }

        return (top + bottom) / 2;
    }
}
