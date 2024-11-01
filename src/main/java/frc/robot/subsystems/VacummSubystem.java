package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacummSubystem extends SubsystemBase {

    private CANSparkMax vacumm;

    public VacummSubystem() {
        vacumm = new CANSparkMax(55, MotorType.kBrushless);
        this.stop();
    }

    public void intakeFull() {
        vacumm.set(1.00);
    }

    public void intakePartial() {
        vacumm.set(0.25);
    }

    public void stop() {
        vacumm.set(0.0);
    }

    public void outtake() {
        vacumm.set(-0.75);
    }
}