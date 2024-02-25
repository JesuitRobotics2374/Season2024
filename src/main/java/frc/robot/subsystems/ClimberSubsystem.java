package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberSubsystem {
    private TalonFX leftClimber;
    private TalonFX rightClimber;

    private final double climbingSpeed = 0.250;

    public ClimberSubsystem() {
        leftClimber = new TalonFX(Constants.LEFT_CLIMBER_MOTOR_ID);
        rightClimber = new TalonFX(Constants.RIGHT_CLIMBER_MOTOR_ID);
        leftClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setNeutralMode(NeutralModeValue.Brake);
    }

    public void startLeftClimberUp() {
        leftClimber.set(climbingSpeed);
    }

    public void startLeftClimberDown() {
        leftClimber.set(-1 * climbingSpeed);
    }

    public void stopLeftClimber() {
        leftClimber.stopMotor();
    }

    public void startRightClimberUp() {
        rightClimber.set(climbingSpeed);
    }

    public void startRightClimberDown() {
        rightClimber.set(-1 * climbingSpeed);
    }

    public void stopRightClimber() {
        rightClimber.stopMotor();
    }
}
