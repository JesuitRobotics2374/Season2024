package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

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
}
