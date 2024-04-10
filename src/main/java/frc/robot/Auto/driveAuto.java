package frc.robot.Auto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.LauncherSubsystem;
import frc.robot.Subsystem.Transfer;
import frc.robot.generated.TunerConstants;
import frc.robot.Subsystem.LauncherSubsystem;

public class driveAuto extends Command {


    private double desiredx;
    private double desiredy;
    private double desiredrotation;

    private CommandSwerveDrivetrain Drive;
      SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(8.18 * 0.1).withRotationalDeadband((1.5 * Math.PI) * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public driveAuto(CommandSwerveDrivetrain Drive,double desiredx,double desiredy,double desiredrotation){
          
        this.Drive = Drive;
    }

    public void initialize(){
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(8.18 * 0.1).withRotationalDeadband((1.5 * Math.PI) * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }
    public void execute(){
        
        Drive.applyRequest(() -> drive.withVelocityX(1 * 8.18) // Drive forward with
        // negative Y (forward)
    .withVelocityY(1* 8.18) // Drive left with negative X (left)
    .withRotationalRate(1 * (1.5 * Math.PI)) // Drive counterclockwise with negative X (left)
    );

    }
    public boolean isFinished(boolean isfinished){
        return isfinished;

    }
    public void end(){

    }

}
