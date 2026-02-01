// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.io.File;
import java.util.Arrays;
import java.util.Set;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{
  /**
   * Swerve drive object.
   */
  
  SwerveDrivePoseEstimator SwerveDrivePoseEstimator;
  private final SwerveDrive swerveDrive;
  private final Pose2d blueAllianceStartingPose = new Pose2d(new Translation2d(Meter.of(1),
                                                                               Meter.of(4)),
                                                             Rotation2d.fromDegrees(0));
  private final Pose2d redAllianceStartingPose = new Pose2d(new Translation2d(Meter.of(16),
                                                                              Meter.of(4)),
                                                            Rotation2d.fromDegrees(180));
                                                            
  private static final Set<Integer> ALLOWED_TAG_IDS = Set.of(9, 10, 25, 26);
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
   public SwerveSubsystem(File directory)
  { 
    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? blueAllianceStartingPose : redAllianceStartingPose;
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible


  }


  @Override
  public void periodic()
  {
 
  }

  @Override
  public void simulationPeriodic()
  {
    
  }


    public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
        swerveDrive::getPose,
        // Robot pose supplier
        swerveDrive::resetOdometry,
        // Method to reset odometry (will be called if your auto has a starting pose)
        swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

    public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
   /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

   public Command stopCommand()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */

     public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  /* 
 public Command driveToLimelightTarget() {
    return runOnce(() -> {
        LimelightHelpers.setPipelineIndex("limelight-raider", 0);
    }).andThen(run(() -> {

        if (!LimelightHelpers.getTV("limelight-raider")) {
            swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            return;
        }

        double tx = LimelightHelpers.getTX("limelight-raider");
        double ty = LimelightHelpers.getTY("limelight-raider");
        double ta = LimelightHelpers.getTA("limelight-raider");

        double kPRot = 0.05;
        double kPForward = 0.2;

        double rotSpeed = -tx * kPRot * -0.6   ;
        double forwardSpeed = -ty * kPForward *-0.5;

        swerveDrive.drive(
            new Translation2d(forwardSpeed, 0),
            rotSpeed,
            false,
            false
        );
        if (ta > 0.35) {

           swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
          
        }

    })).finallyDo(() ->
        swerveDrive.drive(new Translation2d(0, 0), 0, false, false)
    );
}*/

public Command driveToLimelightTarget() {

    final double desiredDistanceMeters = 10.0;

    final double kPForward = 0.4;
    final double kDForward = 0.2;

    final double kPRot = 0.15;
    final double kDRot = 0.0;

    final double maxSearchSpeed = 1.0;

    final double distanceTolerance = 0.2;
    final double xTolerance = 0.03;

    final double stableTimeSeconds = 0.25;

    final double[] lastX = {0.0};
    final double[] prevDistanceError = {0.0};
    final double[] prevXError = {0.0};
    final double[] timeAtSetpoint = {0.0};

    return runOnce(() ->
        LimelightHelpers.setPipelineIndex("limelight-raider", 0)
    ).andThen(
        run(() -> {

            boolean targetVisible =
                LimelightHelpers.getTV("limelight-raider");

            int tagID = (int) LimelightHelpers.getFiducialID("limelight-raider");

            // ---------- FILTER BY ALLOWED TAGS ----------
            if (!targetVisible || !ALLOWED_TAG_IDS.contains(tagID)) {
                double rotSpeed = Math.signum(lastX[0]) * maxSearchSpeed;

                swerveDrive.drive(
                    new Translation2d(0, 0),
                    rotSpeed,
                    false,
                    false
                );
                return;
            }

            double[] pose =
                LimelightHelpers.getTargetPose_RobotSpace(
                    "limelight-raider"
                );

            double x = pose[0];
            double z = pose[2];

            lastX[0] = x;

            double distanceError = desiredDistanceMeters - z;
            double xError = x;

            double distanceDerivative =
                distanceError - prevDistanceError[0];
            double xDerivative =
                xError - prevXError[0];

            prevDistanceError[0] = distanceError;
            prevXError[0] = xError;

            boolean atSetpoint =
                Math.abs(distanceError) < distanceTolerance
                && Math.abs(xError) < xTolerance;

            if (atSetpoint) {
                timeAtSetpoint[0] += 0.02; // ~20ms loop

                swerveDrive.drive(
                    new Translation2d(0, 0),
                    0,
                    false,
                    false
                );
                return;
            } else {
                timeAtSetpoint[0] = 0;
            }

            double forwardSpeed =
                (distanceError * kPForward)
                + (distanceDerivative * kDForward);

            double rotSpeed =
                (-xError * kPRot)
                - (xDerivative * kDRot);

            if (Math.abs(forwardSpeed) < 0.05) forwardSpeed = 0;
            if (Math.abs(rotSpeed) < 0.05) rotSpeed = 0;

            forwardSpeed =
                MathUtil.clamp(forwardSpeed, -1.0, 1.0);
            rotSpeed =
                MathUtil.clamp(rotSpeed, -1.0, 1.0);

            swerveDrive.drive(
                new Translation2d(forwardSpeed * 1.5, 0),
                rotSpeed,
                false,
                false
            );
        })
        .until(() -> timeAtSetpoint[0] >= stableTimeSeconds)
    )
    .finallyDo(() ->
        swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            false,
            false
        )
    );
}


}