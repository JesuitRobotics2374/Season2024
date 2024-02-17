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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class ArmSubsystem extends SubsystemBase {
    ProfiledPIDController armController = new ProfiledPIDController(2, 0.3, 0.1, new Constraints(1, 1.3));
    ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
    TalonFX leftMotor = new TalonFX(Constants.LEFT_ARM_MOTOR_ID);
    TalonFX rightMotor = new TalonFX(Constants.RIGHT_ARM_MOTOR_ID);
    CANcoder encoder = new CANcoder(Constants.ARM_ENCODER_ID, Constants.CAN_BUS_NAME_CANIVORE);
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
    double goal;
    double speed;

    static ArmSubsystem instance;

    public ArmSubsystem() {
        instance = this;
        tab.add("PID Controller", armController);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setInverted(false);
        rightMotor.setControl(new Follower(Constants.LEFT_ARM_MOTOR_ID, true));
        encoder.getConfigurator()
                .apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive).withMagnetOffset(-.454));
        armController.enableContinuousInput(-.5, .5);
        armController.setTolerance(0.01, 0.01);
        goal = encoder.getAbsolutePosition().getValueAsDouble();
        armController.setGoal(goal);
        tab.addDouble("Setpoint", () -> armController.getSetpoint().position);
        tab.addDouble("Point", () -> encoder.getAbsolutePosition().getValueAsDouble());
        tab.addDouble("Motor Power", () -> speed);
    }

    @Override
    public void periodic() {
        speed = Math.min(Math.max(
                armController.calculate(encoder.getAbsolutePosition().getValueAsDouble()), -.30), .30);
        leftMotor.set(speed);
    }

    public void setGoal(double degrees) {
        armController.setGoal(degrees / 360);
    }

    public boolean atGoal() {
        return armController.atGoal();
    }

    public void raise() {
        goal += 0.01;
        armController.setGoal(goal);
    }

    public void lower() {
        goal -= .01;
        armController.setGoal(goal);
    }

    public void resetGoal() {
        goal = encoder.getAbsolutePosition().getValueAsDouble();
        armController.reset(goal);
        armController.setGoal(goal);
    }

    public double angleCalculator(double distance) {
        double top = Math.PI / 2;
        double bottom = 0.0;

        for (int cycle = 0; cycle < 100; cycle++) {
            double angle = (top + bottom) / 2;
            double deltaDistance = distance - Constants.armLength * Math.cos(angle);
            double deltaHeight = Constants.deltaHeight - Constants.armLength * Math.sin(angle);

            double t = deltaDistance / Constants.launchVelocity * Math.cos(angle);
            double y = deltaDistance * Constants.launchVelocity * Math.sin(angle) - 4.903325 * t * t;

            if (y > deltaHeight) {
                top = angle;
            } else if (y < deltaHeight) {
                bottom = angle;
            } else {
                break;
            }
        }

        return (top + bottom) / 2 + Constants.armAngleOffset;
    }

    public void shoot() {
        double angle = angleCalculator(CommandSwerveDrivetrain.getInstance().getDistanceToSpeaker());
        setGoal(angle);
    }

    public static ArmSubsystem getInstance() {
        return instance;
    }
}
