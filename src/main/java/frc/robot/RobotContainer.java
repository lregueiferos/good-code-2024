// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystem.LedSubsystem;
import frc.robot.generated.TunerConstants;



public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final XboxController joystick = new XboxController(0); // My joystick
  private final XboxController operator = new XboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

SlewRateLimiter rampx = new SlewRateLimiter(2);
SlewRateLimiter rampyLimiter = new SlewRateLimiter(2);
SlewRateLimiter ramprotation = new SlewRateLimiter( 4.5);

  XboxController driverXbox = new XboxController(0);
  XboxController operatorxbox = new XboxController(1);
 

  SendableChooser<Command> m_Chooser = new SendableChooser<>();
  
 
  
  
  
 
   

//  public Command getAutonomusCommand(){
//   m_Chooser.addOption("left", new PathPlannerAuto("left"));
//   m_Chooser.addOption("right", new PathPlannerAuto("right"));
//   m_Chooser.addOption("center", new PathPlannerAuto("Test"));
//   m_Chooser.addOption("right leave",new PathPlannerAuto("right leave"));
//   m_Chooser.addOption("NO Auto",null);
  
  
//   return new PathPlannerAuto("Test");
//  }
 
 

  
  LedSubsystem led = new LedSubsystem();
  
  


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);


  private void configureBindings() {
    
      JoystickButton controllerA = new JoystickButton(operator, XboxController.Button.kA.value);
    JoystickButton controlerY = new JoystickButton(operator,XboxController.Button.kY.value );
    //JoystickButton controllerA_2 = new JoystickButton(operatorxbox, XboxController.Button.kA.value);
    //JoystickButton controlerY_2 = new JoystickButton(operatorxbox,XboxController.Button.kY.value );
    JoystickButton controllerB = new JoystickButton(operator, XboxController.Button.kB.value);
    JoystickButton button7 = new JoystickButton(operator, 7);
    JoystickButton controllerX = new JoystickButton(operator, XboxController.Button.kX.value);
    JoystickButton controllerStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    JoystickButton controllerRightbump = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    JoystickButton controllerLeftbump = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    POVButton operatorDpadUp = new POVButton(operator, 0);
    JoystickButton driverTrigger = new JoystickButton(joystick, 1);
     JoystickButton button11 = new JoystickButton(joystick, 11);
     JoystickButton button2 = new JoystickButton(joystick, 2);
     JoystickButton driver6= new JoystickButton(joystick, 6);
      
    //led.setAnimation();
    DriverStation.Alliance color;
    boolean red = false;
	  var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                red = alliance.get() == DriverStation.Alliance.Red;
              }

    if(red){
      led.setColorred();
    }else{
      led.setcolorblue();
    }
    //DRIVER COMMANDS

    //intake down and up command


    // button2.onTrue(new InstantCommand(() ->intake.moveIntake(0, 0, false)));
    // button2.onTrue(new InstantCommand(() ->intake.moveSolenoid(false)));
    //button2.onTrue(new SequentialCommandGroup( new InstantCommand(() ->transfer.activatetransfer(-.4)).withTimeout(.1), new InstantCommand(() ->transfer.activatetransfer(0))  ));

    
    //zeros gyro
    //new JoystickButton(driverXbox, 11).onTrue((new InstantCommand(drivebase::zeroGyro)));


//OPERATOR COMMANDS

    //fire a note - shooter or Amp
  


    // TROUBLESHOOTING COMMANDS

    //Brings up Intake and shooter on toggle
    

    //controllerB.onTrue(new InstantCommand(() -> shoot.moveLauncher(-1, 1, 0)));
    //controllerB.onFalse(new InstantCommand(() -> shoot.moveLauncher(0, 0, 0)));
    //climber.setPower(operator.getRawAxis(5));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX( rampx.calculate( MathUtil.applyDeadband(joystick.getLeftX()*((joystick.getRawAxis(3)+1)/2),.1)) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(rampyLimiter.calculate(MathUtil.applyDeadband(-joystick.getLeftY()*((joystick.getRawAxis(3)+1)/2),.1)) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(ramprotation.calculate(MathUtil.applyDeadband(-joystick.getRawAxis(2)*((joystick.getRawAxis(3)+1)/2),.3)) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
    //.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    button11.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
  
 
    SmartDashboard.putData(m_Chooser);

    


    configureBindings();
  }

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
    //return new PathPlannerAuto("center");
    //return Commands.print("No autonomous command configured");
    //return new SequentialCommandGroup( new InstantCommand(() ->intake.moveSolenoid(true)).withTimeout(.5),new InstantCommand(() ->shoot.moveSolenoid(true)).withTimeout(.5),
    //new InstantCommand(() ->shoot.moveLauncher(-.6, .7, 0)).withTimeout(2),new WaitCommand(3), new InstantCommand(() ->transfer.activatetransfer(.8)).withTimeout(.5), new WaitCommand(1),new InstantCommand(() ->intake.moveSolenoid(false)).withTimeout(.5)
    //, new InstantCommand(() ->shoot.moveLauncher(0, 0, 0)).withTimeout(2),new InstantCommand(() ->shoot.moveSolenoid(false)).withTimeout(.5) );
  }
}
