package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class ArmSubsystem extends SubsystemBase {
    ProfiledPIDController armController = new ProfiledPIDController(4, 0.25, 0, new Constraints(1.5, 1.6));
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
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive).withMagnetOffset(-.411));
        armController.enableContinuousInput(-.5, .5);
        armController.setTolerance(0.007, 0.02);
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
        // if (!armController.atGoal()) {
        // if (speed < 0) {
        // speed = Math.min(speed, -.015);
        // } else if (speed > 0) {
        // speed = Math.max(speed, 0.015);
        // }
        // }
        leftMotor.setVoltage(speed * 12 + Constants.FEED_FORWARD_VOLTAGE
                * Math.sin(encoder.getAbsolutePosition().getValueAsDouble() * Math.PI * -2));
    }

    public void setGoal(double degrees) {
        armController.setGoal(degrees / 360);
        goal = degrees / 360;
    }

    public boolean atGoal() {
        return armController.atGoal();
    }

    public void raise() {
        if (goal < Constants.FORWARD_SOFT_STOP) {
            goal += 0.01;
            armController.setGoal(goal);
        }

    }

    public void lower() {
        if (goal > -0.242) {
            goal -= .01;
            armController.setGoal(goal);
        }
    }

    public void resetGoal() {
        goal = encoder.getAbsolutePosition().getValueAsDouble();
        armController.reset(goal);
        armController.setGoal(goal);
    }

    public double angleCalculator(double distance) {
        double top = Math.PI / 2;
        double bottom = 0.0;

        for (int cycle = 0; cycle < 50; cycle++) {
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

    public double myAngleCalculator(double distance) {
        double top = Math.PI / 2;
        double bottom = Math.PI;

        for (int cycle = 0; cycle < 11; cycle++) {
            double angle = (top + bottom) / 2;
            double deltaDistance = distance - Constants.armLength * Math.cos(angle);
            double deltaHeight = Constants.deltaHeight - Constants.armLength * Math.sin(angle);

            double t = deltaDistance / (Constants.launchVelocity * Math.cos(angle - Constants.armAngleOffset));
            double y = Constants.launchVelocity * Math.sin(angle - Constants.armAngleOffset) * t - 4.903325 * t * t;
            // System.out.println(angle);
            if (y > deltaHeight) {
                bottom = angle;
            } else if (y < deltaHeight) {
                top = angle;
            } else {
                break;
            }
        }

        return Math.toDegrees((Math.PI / 2) - ((top + bottom) / 2));
    }

    public void shoot() {
        // System.out.println(CommandSwerveDrivetrain.getInstance().getDistanceToSpeaker());
        double angle = myAngleCalculator(CommandSwerveDrivetrain.getInstance().getDistanceToSpeaker());
        setGoal(angle);
        // System.out.println(angle / 360);
    }

    public static ArmSubsystem getInstance() {
        return instance;
    }
}
