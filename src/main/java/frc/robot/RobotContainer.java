// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.FeetPerSecond;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.io.File;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import swervelib.SwerveInputStream;
import yams.units.YUnits;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed

  final         CommandXboxController driverXbox = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final IntakeSubsystem       Intake     = new IntakeSubsystem();
  private final ShooterSubsystem      Shooter    = new ShooterSubsystem();
  private final KickerSubsystem                Kicker     = new KickerSubsystem();
  private final IndexerSubsystem               Indexer    = new IndexerSubsystem();
  private final ClimberSubsystem               Climber    = new ClimberSubsystem();
  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */

//Setting default commands
     public void defaultCommands(){
    Intake.setDefaultCommand(Intake.stopIntakeCommand());
    Intake.setDefaultCommand(Intake.setAngle(Degrees.of(90)));
    Shooter.setDefaultCommand(Shooter.setVelocity(FeetPerSecond.of(0)));
    Indexer.setDefaultCommand(Indexer.stopIndexerCommand());
    Kicker.setDefaultCommand(Kicker.stopKickerCommand());
     }
  SwerveInputStream driveRobotOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> driverXbox.getLeftY() * 0.7,
                                                              () -> driverXbox.getLeftX() * 0.7)
                                                          .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                          4)*-0.7)
                                                          .deadband(OperatorConstants.DEADBAND)
                                                          .scaleTranslation(0.6)
                                                          .robotRelative(true);



     SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 0.7,
                                                                () -> driverXbox.getLeftX() * 0.7)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.6)
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   
  public RobotContainer()
  {

    // NamedCommands.registerCommand("DriveToLimelight", drivebase.driveToLimelightTarget());
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
    DriverStation.silenceJoystickConnectionWarning(true);

    drivebase.setupPathPlanner();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Straight",AutoBuilder.buildAuto("Straight Auto")
);
    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //Add this back to make it robot Oriented

    //Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);  
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);

    //ROBOT ORIENTED DRIVE
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // LIMELIGHT CONTROLS
    driverXbox.rightBumper().whileTrue(drivebase.driveToLimelightTarget());

  
    // INTAKE CONTROLS
    driverXbox.leftTrigger().whileTrue(Intake.setAngle(Degrees.of(90)));
    driverXbox.leftTrigger().whileFalse(Intake.setAngle(Degrees.of(10)));
    driverXbox.leftTrigger().whileTrue(Intake.runIntakeCommand(Constants.IntakeConstants.IntakeSpeed));
    driverXbox.leftTrigger().whileFalse(Intake.stopIntakeCommand());
    driverXbox.pov(90).whileTrue(Intake.sysId());
    
    // SHOOTER CONTROLS

    driverXbox.rightTrigger().whileTrue(Shooter.setVelocity(FeetPerSecond.of(1)));
    driverXbox.rightTrigger().whileFalse(Shooter.setVelocity(FeetPerSecond.of(0)));

    //KICKER CONTROLS
    
     driverXbox.b().whileTrue(Kicker.runKickerCommand(Constants.KickerConstants.KickSpeed));
     driverXbox.b().whileFalse(Kicker.stopKickerCommand());

     //INDEXER CONTROLS

     driverXbox.b().whileTrue(Indexer.runIndexerCommand(Constants.IndexConstants.IndexSpeed));
     driverXbox.b().whileFalse(Indexer.stopIndexerCommand());

     driverXbox.pov(180).whileTrue(Climber.setAngle(Degrees.of(180)));
     driverXbox.pov(180).whileFalse(Climber.setAngle(Degrees.of(90)));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    //return autoChooser.getSelected();
    return drivebase.getAutonomousCommand("Do Nothing");
  }

}