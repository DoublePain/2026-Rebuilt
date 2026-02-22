package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import com.revrobotics.spark.SparkAbsoluteEncoder;



public class IntakeSubsystem  extends SubsystemBase {
    
     private final SparkMax                IntakearmMotor    = new SparkMax(Constants.IDConstants.IntakearmMotor_ID, MotorType.kBrushless);
      private final SparkMax               IntakeMotor    = new SparkMax(Constants.IDConstants.IntakeMotor_ID, MotorType.kBrushless);
      private final SparkAbsoluteEncoder absEncoder =
                                            IntakearmMotor.getAbsoluteEncoder();
      private static final double ABSOLUTE_OFFSET_DEG = 0.0; //Unknown currently
  

  double AbsoluteAngle = (absEncoder.getPosition() * 360.0) - ABSOLUTE_OFFSET_DEG;

    
      //Will tune this on Bot, Sim isnt being nice with YAMS arm tuning
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(5, 0, 0.1, DegreesPerSecond.of(20), DegreesPerSecondPerSecond.of(20))  
      .withSoftLimit(Degrees.of(5), Degrees.of(90))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5,5)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0.11044, 0, 0, 0.55929))
      .withControlMode(ControlMode.CLOSED_LOOP).withExternalEncoder(absEncoder);
  private final SmartMotorController       motor            = new SparkWrapper(IntakearmMotor,
                                                                              DCMotor.getNEO(1),
                                                                              motorConfig);
  
  private       ArmConfig m_config = new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(0), Degrees.of(100))
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(15))
      .withStartingPosition(Degrees.of(0));

  private final Arm       intakeArm      = new Arm(m_config);

  

  public IntakeSubsystem()
  {

  }

  public void periodic()
  {
    intakeArm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    intakeArm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return intakeArm.set(dutycycle);
  }

  public Command sysId()
  {
    return intakeArm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return intakeArm.setAngle(angle);
  }
  
   public Command DeployIntake(Angle angle)
  {
    return intakeArm.setAngle(Degrees.of(25));  //DONT KNOW IF 25 IS RIGHT YET
  }

   public Command StowIntake(Angle angle)
  {
    return intakeArm.setAngle(Degrees.of(115));  //DONT KNOW IF 25 IS RIGHT YET
  }
  

// Run intake roller
public void runIntake(double speed) {
    IntakeMotor.set(speed);
}

// Stop intake roller
public void stopIntake() {
    IntakeMotor.set(0);
}

// Command to run intake
public Command runIntakeCommand(double speed) {
    return run(() -> runIntake(speed));
}

// Command to stop intake
public Command stopIntakeCommand() {
    return run(this::stopIntake);
}

public Command deployAndSpinCommand() {
    return Commands.parallel(
        DeployIntake(Degrees.of(25)),
        run(() -> runIntake(Constants.IntakeConstants.IntakeSpeed))
    ).finallyDo(() -> {
        stopIntake();
        StowIntake(Degrees.of(115)).schedule();
    }).withName("Intake.DeployAndSpin");
}

}



