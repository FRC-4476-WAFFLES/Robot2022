package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.RobotContainer.*;

import java.util.ArrayList;
import java.util.List;

public class DriveAuto extends SwerveControllerCommand {
  static PIDController xController = new PIDController(-7.1, 0.0, -0.1); // Original -5.0, 0.0, -0.1
  static PIDController yController = new PIDController(7.1, 0.0, 0.1);
  static ProfiledPIDController thetaController = new ProfiledPIDController(-6.0, 0.0, 0.0,
    new TrapezoidProfile.Constraints(
      Constants.SwerveConstants.maxAttainableRotationRateRadiansPerSecond, 
      Constants.SwerveConstants.maxAccelerationMetersPerSecondSquared));
  public static Timer timer = new Timer();
  
  private SwervePath swervePath;
  
  public DriveAuto(SwervePath swervePath) {
    super(swervePath.trajectory,
      () -> driveSubsystem.getOdometryLocation(),
      driveSubsystem.kinematics,
      xController, yController,
      thetaController,
      () -> swervePath.sampleAngle(timer.get()),
      (states) -> driveSubsystem.setModuleStates(states),
      driveSubsystem);
    
    this.swervePath = swervePath;
  }
  
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    super.initialize();
  }
  
  @Override
  public void execute() {
    if(Robot.isSimulation()) {
      State state = swervePath.trajectory.sample(timer.get());
      Rotation2d angle = swervePath.sampleAngle(timer.get());
      driveSubsystem.field.setRobotPose(state.poseMeters.getX(), state.poseMeters.getY(), angle);
    }

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveSubsystem.stop();
  }

  /** 
   * SwervePath consists of a normal trajectory and also the directions it
   * should face as it drives. Right now it will treat the angles as being
   * separate from the points, so the angles will not match up with the moves.
   */
  public static class SwervePath {
    private Pose2d startPose;
    private List<Translation2d> points = new ArrayList<Translation2d>();
    private List<Rotation2d> angles = new ArrayList<Rotation2d>();
    Trajectory trajectory;
    
    /**
     * Create a swerve path with a starting pose.
     * @param x The X coordinate of the starting point.
     * @param y The Y coordinate of the starting point.
     * @param angle The direction the robot should face.
     * @param pathHeading The direction the robot should start its motion in.
     */
    public SwervePath(double x, double y, double angle, double pathHeading) {
      startPose = new Pose2d(x, y, Rotation2d.fromDegrees(pathHeading));
      angles.add(Rotation2d.fromDegrees(angle));
    }
      
    /**
     * Add a waypoint to this path. The robot will pass through this point.
     * @param x The X coordinate of the waypoint.
     * @param y The Y coordinate of the waypoint.
     * @param angle The angle the robot should face.
     * @return Returns this, so that methods can be chained.
     */
    public SwervePath waypoint(double x, double y, double angle) {
      this.points.add(new Translation2d(x, y));
      this.angles.add(Rotation2d.fromDegrees(angle));
      
      return this;
    }
  
    /**
     * Add the ending pose to the SwervePath, generating the trajectory.
     * @param x The X coordinate of the end pose.
     * @param y The Y coordinate of the end pose.
     * @param angle The angle the robot should face.
     * @param pathHeading The direction the robot should be moving while it comes to a stop.
     * @param maxSpeedM The maximum speed to travel along this path.
     * @return The finished swerve path.
     */
    public SwervePath finish(double x, double y, double angle, double pathHeading, double maxSpeedM) {
      this.angles.add(Rotation2d.fromDegrees(angle));

      TrajectoryConfig config = new TrajectoryConfig(
        maxSpeedM,
        Constants.SwerveConstants.maxAccelerationMetersPerSecondSquared);
      config.addConstraint(new SwerveDriveKinematicsConstraint(driveSubsystem.kinematics, maxSpeedM));
      
      this.trajectory = TrajectoryGenerator.generateTrajectory(startPose, points, new Pose2d(x, y, Rotation2d.fromDegrees(pathHeading)), config);
      return this;
    }

    /**
     * Returns the facing angle at the given time.
     * @param timeSeconds The time to sample at.
     * @return The angle the robot should face at that time.
     */
    Rotation2d sampleAngle(double timeSeconds) {
      // There's no easy way to get the actual time the robot passes through
      // each point, so we will aproximate by saying that each point is an
      // equal time apart.
      double totalTime = this.trajectory.getTotalTimeSeconds();
      int numStates = this.angles.size();
      
      if(timeSeconds <= 0) {
        return this.angles.get(0);
      }
      if(timeSeconds >= totalTime) {
        return this.angles.get(numStates - 1);
      }

      double progress = (timeSeconds / totalTime) * (numStates - 1);
      int index = (int) progress;
      double fractional = progress - (double) index;

      Rotation2d startAngle = this.angles.get(index);
      Rotation2d endAngle = this.angles.get(index + 1);
      return startAngle.interpolate(endAngle, fractional);
    }
  }
}
