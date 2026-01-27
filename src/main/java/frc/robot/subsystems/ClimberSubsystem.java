package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

public class ClimberSubsystem extends SubsystemBase
{

  private final SparkMax               ClimberMotor    = new SparkMax(Constants.IDConstants.ClimberMotor_ID, MotorType.kBrushless);
  
   //Will tune this on Bot, Sim isnt being nice with YAMS arm tuning
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(50, 0, 1, DegreesPerSecond.of(10), DegreesPerSecondPerSecond.of(10))
      .withSoftLimit(Degrees.of(210), Degrees.of(90))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor            = new SparkWrapper(ClimberMotor,
                                                                              DCMotor.getNEO(1),
                                                                              motorConfig);
  
  private       ArmConfig m_config = new ArmConfig(motor)
      .withLength(Meters.of(0.35))
      .withHardLimit(Degrees.of(245), Degrees.of(75))
      .withTelemetry("Climber", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(145))
      .withStartingPosition(Degrees.of(90));

  private final Arm       Climber      = new Arm(m_config);

  public ClimberSubsystem()
  {
  }

  public void periodic()
  {
    Climber.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    Climber.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return Climber.set(dutycycle);
  }

  public Command sysId()
  {
    return Climber.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return Climber.setAngle(angle);
  }
}
