package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {

    private final SparkMax kickerMotor =
        new SparkMax(Constants.IDConstants.KickerMotor_ID, MotorType.kBrushless);

    // Run kicker motor
    public void runKicker(double speed) {
        kickerMotor.set(speed);
    }

    // Stop kicker motor
    public void stopKicker() {
        kickerMotor.set(0);
    }

    // Command to run kicker at a speed
    public Command runKickerCommand(double speed) {
        return run(() -> runKicker(speed));
    }

    // Command to stop kicker
    public Command stopKickerCommand() {
        return run(this::stopKicker);
    }

    //Gimme all that telemtry 
     @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "KickerRPM",
            kickerMotor.getEncoder().getVelocity()
        );
    }
}