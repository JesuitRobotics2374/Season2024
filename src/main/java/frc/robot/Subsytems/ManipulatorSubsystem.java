package frc.robot.Subsytems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkFlex intakeMotor;
    private final CANSparkFlex shootMotor;
    private Boolean intakeMode; 
    private Boolean shootMode; 

    public ManipulatorSubsystem(RobotContainer container){
        intakeMotor = new CANSparkFlex(0, MotorType.kBrushless);
        shootMotor = new CANSparkFlex(1, MotorType.kBrushless);
        intakeMode = false; 
        shootMode = false; 
    }

   
       //can object name it global, create it, method running stoping 
    /* 
    private void activate() {
        active = true;
    }*/


    public void intake() {
        if(intakeMode = true){
            intakeMotor.set(0.3);
        
        }

    }

    /**
     * Starts outtaking
     */
    public void shoot() {
        if(intakeMode  = true){
            shootMode = true;
            shootMotor.set(0.3);
            //activate();
        }
    }
       
    

    /**
     * Stops taking
     */
    public void stoptake() {
        intakeMotor.stopMotor();
        shootMotor.stopMotor();
        
    }
}



