package frc.robot.commands.unused;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

public class IntakeCommand extends Command {
    ManipulatorSubsystem m_ManipulatorSubsystem;

    public IntakeCommand(ManipulatorSubsystem manipulatorSubsystem) {
        m_ManipulatorSubsystem = manipulatorSubsystem;
        addRequirements(m_ManipulatorSubsystem);
    }

    @Override
    public void initialize() {
        m_ManipulatorSubsystem.intake();
    }

    @Override
    public boolean isFinished() {
        return !m_ManipulatorSubsystem.getIntake();
    }

    @Override
    public void end(boolean interrupted) {
        CommandSwerveDrivetrain.getInstance().setControl(new SwerveRequest.SwerveDriveBrake());
        ArmSubsystem.getInstance().setGoal(Constants.HOLD_NOTE * 360);
    }
}
