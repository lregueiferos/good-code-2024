package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.LauncherSubsystem;
import frc.robot.Subsystem.Transfer;

public class IntakeStop extends Command {
        private final IntakeSubsystem intakeSubsystem;
        private final Transfer transfer;
        

    public IntakeStop(IntakeSubsystem intakeSubsystem,Transfer transfer){
        this.intakeSubsystem = intakeSubsystem;
        this.transfer = transfer;
        
        addRequirements(intakeSubsystem);
        addRequirements(transfer);
        
    }
     public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    intakeSubsystem.moveIntake(1, 1, true);
    //transfer.activatetransfer(1);


    
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return (intakeSubsystem.intake_limit());
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    intakeSubsystem.moveIntake(0, 0, false);
    intakeSubsystem.moveSolenoid(false);
    // //transfer.activatetransfer(-.4)).withTimeout(.1)
    transfer.activatetransfer(0);
  }
}

    

