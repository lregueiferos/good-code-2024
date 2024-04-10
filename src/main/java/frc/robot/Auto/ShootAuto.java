package frc.robot.Auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystem.Transfer;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootAuto extends SequentialCommandGroup {
    
    private IntakeSubsystem IntakeSubsystem;
    private LauncherSubsystem LauncherSubsystem;
    private CommandSwerveDrivetrain CommandSwerveDrivetrain;
    private Transfer Transfer;
    
    public ShootAuto (IntakeSubsystem IntakeSubsystem, LauncherSubsystem LauncherSubsystem,Transfer Transfer){
        this.IntakeSubsystem = IntakeSubsystem;
        this.LauncherSubsystem = LauncherSubsystem;
        this.Transfer = Transfer;

        addCommands(
            //new InstantCommand(() ->Transfer.activatetransfer(-.8)).withTimeout(.5),
            //new WaitCommand(0.02),
            new InstantCommand(() ->IntakeSubsystem.moveSolenoid(true)),
            new InstantCommand(() ->LauncherSubsystem.moveSolenoid(true)).withTimeout(.5),
            new InstantCommand(() ->LauncherSubsystem.moveLauncher(-.6, .7, 0)).withTimeout(2),
            new WaitCommand(1.1),
            new InstantCommand(() ->Transfer.activatetransfer(.8)).withTimeout(.5),
            new WaitCommand(0.7),new InstantCommand(() ->IntakeSubsystem.moveSolenoid(false)).withTimeout(.5),
            new InstantCommand(() ->LauncherSubsystem.moveLauncher(0, 0, 0)).withTimeout(2),
            new InstantCommand(() ->LauncherSubsystem.moveSolenoid(false)).withTimeout(.5) );
            

    }

}
