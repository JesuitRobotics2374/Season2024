package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VacummSubystem;

public class SubsystemAction extends Command {

    private VacummSubystem vac;
    private ArmSubsystem arm;
    private String action;

    public SubsystemAction(VacummSubystem vac, ArmSubsystem arm, String action) {
        this.vac = vac;
        this.arm = arm;
        this.action = action;
    }

    @Override
    public void initialize() {
        if (action.equals("intake")) {
            vac.intakePartial();
        } else if (action.equals("stop")) {
            vac.stop();
        } else if (action.equals("outtake")) {
            vac.outtake();
        } else if (action.equals("lower")) {
            arm.setGoal(-0.2 * 360);
            System.out.println("lowering...");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
