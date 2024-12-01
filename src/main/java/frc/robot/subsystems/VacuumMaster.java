package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacuumMaster extends SubsystemBase {

    private VacummSubystem vac1;
    private VacummSubystem vac2;
    private VacummSubystem vac3;

    private VacummSubystem targetVac;

    public VacuumMaster(VacummSubystem vac1, VacummSubystem vac2, VacummSubystem vac3) {
        this.vac1 = vac1;
        this.vac2 = vac2;
        this.vac3 = vac3;
        targetVac = vac1;
    }

    public void intakeFull() {
        targetVac.intakeFull();
    }

    public void intakePartial() {
        targetVac.intakePartial();
    }

    public void stop() {
        targetVac.stop();
    }

    public void outtake() {
        targetVac.outtake();
    }

    public int getTargetVacAsInt() {
        if (targetVac == vac1) {
            return 1;
        } else if (targetVac == vac2) {
            return 2;
        } else {
            return 3;
        }
    }

    public VacummSubystem getTargetVac() {
        return targetVac;
    }

    public VacummSubystem[] getVacs() {
        return new VacummSubystem[] { vac1, vac2, vac3 };
    }

    public void setTargetVac(int vac) {
        if (vac == 1) {
            targetVac = vac1;
        } else if (vac == 2) {
            targetVac = vac2;
        } else if (vac == 3) {
            targetVac = vac3;
        } else {
            throw new IllegalArgumentException("VacuumMaster.setTargetVac requires an integer between 1 and 3");
        }
    }

}