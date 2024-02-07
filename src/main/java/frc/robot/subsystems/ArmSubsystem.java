package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    ProfiledPIDController armController = new ProfiledPIDController(0.5, 0.1, 0.05, null);
    ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
    TalonFX leftMotor = new TalonFX(Constants.LEFT_ARM_MOTOR_ID);
    TalonFX rightMotor = new TalonFX(Constants.RIGHT_ARM_MOTOR_ID);
    CANcoder encoder = new CANcoder(Constants.ARM_ENCODER_ID);

    public ArmSubsystem() {
        rightMotor.setControl(new Follower(Constants.LEFT_ARM_MOTOR_ID, true));
    }

    public void setGoal(double degrees) {
        armController.setGoal(degrees / 360);
    }

    @Override
    public void periodic() {
        leftMotor.set(
                armController.calculate(encoder.getAbsolutePosition().getValueAsDouble() + armFeedforward.calculate(
                        encoder.getAbsolutePosition().getValueAsDouble(), encoder.getVelocity().getValueAsDouble())));
    }
}
