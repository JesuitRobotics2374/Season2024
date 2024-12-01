package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacummSubystem extends SubsystemBase {

    private CANSparkMax vacumm;

    private String state = "Stopped";

    public VacummSubystem(int id) {
        vacumm = new CANSparkMax(id, MotorType.kBrushless);
        System.out.println("Created VAC: " + vacumm.getDeviceId());
        this.stop();
    }

    public void intakeFull() {
        vacumm.set(1.00);
        state = "Intake Max";
    }

    public void intakePartial() {
        vacumm.set(0.50);

        state = "Intake Partial";

    }

    public void stop() {
        vacumm.set(0.0);
        state = "Stopped";
    }

    public void outtake() {
        vacumm.set(-0.75);
        state = "Outtake";
    }

    public String getState() {
        return state;
    }
}