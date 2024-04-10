package frc.robot.Subsystem;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase{
private TalonFX TransferMotor1;
private TalonFX TransferMotor2;
 

    
public Transfer (TalonFX TransferMotor1)
  {
    this.TransferMotor1 = TransferMotor1;
    
    
  }
  public void activatetransfer(double TransferSpeed1){
    TransferMotor1.set(TransferSpeed1);

 
    
  }
}

