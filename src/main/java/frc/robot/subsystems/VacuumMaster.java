package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacuumMaster extends SubsystemBase {

    private VacummSubystem vac1;
    private VacummSubystem vac2;
    private VacummSubystem vac3;

    private VacummSubystem targetVac;

    private boolean allVacs;

    public VacuumMaster(VacummSubystem vac1, VacummSubystem vac2, VacummSubystem vac3) {
        this.vac1 = vac1;
        this.vac2 = vac2;
        this.vac3 = vac3;
        targetVac = vac1;
        allVacs = false;
    }

    public void intakeFull() {
        if (allVacs) {
            vac1.intakeFull();
            vac2.intakeFull();
            vac3.intakeFull();
            return;
        }
        targetVac.intakeFull();
    }

    public void intakePartial() {
        if (allVacs) {
            vac1.intakePartial();
            vac2.intakePartial();
            vac3.intakePartial();
            return;
        }
        targetVac.intakePartial();
    }

    public void stop() {
        if (allVacs) {
            vac1.stop();
            vac2.stop();
            vac3.stop();
            return;
        }
        targetVac.stop();
    }

    public void outtake() {
        if (allVacs) {
            vac1.outtake();
            vac2.outtake();
            vac3.outtake();
            return;
        }
        targetVac.outtake();
    }

    public String getTargetVacAsString() {
        if (allVacs) {
            return "ALL VACUUMS";
        } else
        if (targetVac == vac1) {
            return "Black";
        } else if (targetVac == vac2) {
            return "Green";
        } else {
            return "White";
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
            allVacs = false;
        } else if (vac == 2) {
            targetVac = vac2;
            allVacs = false;
        } else if (vac == 3) {
            targetVac = vac3;
            allVacs = false;
        } else {
            throw new IllegalArgumentException("VacuumMaster.setTargetVac requires an integer between 1 and 3");
        }
    }

    public void targetAll() {
        allVacs = true;
    }

}