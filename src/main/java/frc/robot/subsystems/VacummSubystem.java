package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacummSubystem extends SubsystemBase {

    private CANSparkMax vacumm;

    public VacummSubystem() {
        vacumm = new CANSparkMax(55, MotorType.kBrushless);
        this.intake();
    }

    public void intake() {
        vacumm.set(0.75);
        System.out.println("intkae");
    }

    public void stop() {
        vacumm.set(0.0);
        System.out.println("stop");
    }

    public void outtake() {
        vacumm.set(-0.75);
        System.out.println("outtake");
    }
}
