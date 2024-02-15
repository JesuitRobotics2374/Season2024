package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class BasicCommand extends Command {

    public BasicCommand() {

    }

    @Override
    public void initialize() {
        System.out.println("SOMETHING HAPPENS HERE!");
    }

}
