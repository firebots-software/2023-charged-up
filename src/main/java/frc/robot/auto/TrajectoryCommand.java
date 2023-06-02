package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Custom PathPlanner version of SwerveControllerCommand */
public class TrajectoryCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final SwerveDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final boolean useAllianceColor;

  private final Supplier<Rotation2d> desiredRotation;
  private final Supplier<Boolean> wantsVisionRotationAlign;
  private final Supplier<Boolean> wantsVisionTranslationAlign;

  private PathPlannerTrajectory transformedTrajectory;

  private boolean targetDetected;
  private double alignWaitTime;

  private static SwerveSubsystem swerve;
  private static PhotonVision pv;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError =
      TrajectoryCommand::defaultLogError;

  

  /**
   * Constructs a new TrajectoryCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param wantsVisionTranslationAlign Whether the trajectory ends with a vision target.
   * @param alignWaitTime Time to wait BEFORE beginning vision alignment process. Useful for not overriding the generated path preemptively.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  public TrajectoryCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Supplier<Boolean> wantsVisionRotationAlign,
      Supplier<Boolean> wantsVisionTranslationAlign,
      double alignWaitTime,
      boolean useAllianceColor,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.kinematics = kinematics;
    this.controller = new HolonomicDriveController(xController, yController, rotationController);
    this.outputModuleStates = outputModuleStates;
    this.desiredRotation = () -> trajectory.getEndState().poseMeters.getRotation();
    this.wantsVisionRotationAlign = wantsVisionRotationAlign;
    this.wantsVisionTranslationAlign = wantsVisionTranslationAlign;
    this.useAllianceColor = useAllianceColor;

    this.targetDetected = false;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new TrajectoryCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param wantsVisionTranslationAlign Whether the trajectory ends with a vision target.
   * @param alignWaitTime Time to wait BEFORE beginning vision alignment process. Useful for not overriding the generated path preemptively.
   * @param requirements The subsystems to require.
   */
  public TrajectoryCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Supplier<Boolean> wantsVisionRotationAlign,
      Supplier<Boolean> wantsVisionTranslationAlign,
      double alignWaitTime,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        kinematics,
        xController,
        yController,
        rotationController,
        outputModuleStates,
        wantsVisionRotationAlign,
        wantsVisionTranslationAlign,
        alignWaitTime,
        false,
        requirements);
  }

  @Override
  public void initialize() {
    if (useAllianceColor && trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    timer.reset();
    timer.start();
    swerve.resetOdometry(this.trajectory.sample(0).poseMeters);

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();

    // Necessary PPServer material, not important to vision alignment

    PathPlannerState desiredPPState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    Pose2d currentPose = this.poseSupplier.get();
    PathPlannerServer.sendPathFollowingData(
        new Pose2d(desiredPPState.poseMeters.getTranslation(), desiredPPState.holonomicRotation),
        currentPose);

    

    var desiredState = this.trajectory.sample(currentTime);
    Rotation2d desiredRotation = this.desiredRotation.get();
    
    // if (this.wantsVisionRotationAlign.get() && pv.hasTarget(pv.getLatestPipeline())){
    //     desiredRotation = Rotation2d.fromDegrees(
    //         pv.getYaw(pv.getBestTarget(pv.getLatestPipeline())));    
    // } else {
    //     desiredRotation = this.desiredRotation.get();
    // }



    // theoretical translation alignment
    
if(!this.targetDetected){
    if(timer.hasElapsed(alignWaitTime) && this.wantsVisionTranslationAlign.get() && pv.hasTarget(pv.getLatestPipeline())){
      double forwardDistToTarget = pv.getX();
      double leftwardDistToTarget = pv.getY();
      Translation2d distToTarget = new Translation2d(forwardDistToTarget, leftwardDistToTarget);
      Rotation2d initialHeading = new Rotation2d(forwardDistToTarget, leftwardDistToTarget);
      Rotation2d angleToTarget = Rotation2d.fromDegrees(-pv.getYaw(pv.getBestTarget(pv.getLatestPipeline())));
      transformedTrajectory = 
        PathPlanner.generatePath(
          new PathConstraints(AutonConstants.kVMax, AutonConstants.kAMax),
          new PathPoint(currentPose.getTranslation(), initialHeading, currentPose.getRotation(), desiredState.velocityMetersPerSecond),
          new PathPoint(currentPose.getTranslation().plus(distToTarget), angleToTarget)
        );
      timer.restart();
      this.targetDetected = true;
      }   
    }

    // PID calculation and setting swerve module states

    ChassisSpeeds targetChassisSpeeds =
    this.controller.calculate(this.poseSupplier.get(), desiredState, desiredRotation);
    SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

    this.outputModuleStates.accept(targetModuleStates);
    
    // Logging

    if (logTargetPose != null) {
      logTargetPose.accept(
          new Pose2d(desiredState.poseMeters.getTranslation(), desiredPPState.holonomicRotation));
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredPPState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredPPState.holonomicRotation));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
        this.outputModuleStates.accept(
            this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPSwerveControllerCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    TrajectoryCommand.logActiveTrajectory = logActiveTrajectory;
    TrajectoryCommand.logTargetPose = logTargetPose;
    TrajectoryCommand.logSetpoint = logSetpoint;
    TrajectoryCommand.logError = logError;
  }
}
