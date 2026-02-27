package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSpinSubsystem;
import static edu.wpi.first.units.Units.Degrees;
import frc.robot.Constants;

public class DeployAndSpinIntake extends SequentialCommandGroup {

    public DeployAndSpinIntake(IntakeSubsystem arm, IntakeSpinSubsystem spin) {

        addCommands(
            // Deploy arm and spin wheels in parallel
            new ParallelCommandGroup(
                arm.DeployIntake(Degrees.of(25)),                  
                new InstantCommand(() -> spin.runIntake(Constants.IntakeConstants.IntakeSpeed)) 
            ),

            // Stop wheels and stow arm
            new InstantCommand(() -> {
                spin.stopIntake();                   // stop wheels
                arm.StowIntake(Degrees.of(115)).schedule(); // stow arm
            })
        );
    }
}