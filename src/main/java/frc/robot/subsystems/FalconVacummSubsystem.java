
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FalconVacummSubsystem extends SubsystemBase {

    private TalonFX vacumm;

    public FalconVacummSubsystem() {
        vacumm = new TalonFX(19);
        this.stop();
    }

    public void intakeFull() {
        vacumm.set(1.00);
    }

    public void intakePartial() {
        vacumm.set(0.08);
    }

    public void stop() {
        vacumm.set(0.0);
    }

    public void outtake() {
        vacumm.set(-0.08);
    }
}
