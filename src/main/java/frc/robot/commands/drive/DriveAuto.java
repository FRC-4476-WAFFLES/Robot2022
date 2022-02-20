package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

import java.util.List;

public class DriveAuto extends SwerveControllerCommand {
  static PIDController xController = new PIDController(-5.0, 0.0, -0.1);
  static PIDController yController = new PIDController(5.0, 0.0, 0.1);
  static ProfiledPIDController thetaController = new ProfiledPIDController(-2.0, 0.0, 0.0,
    new TrapezoidProfile.Constraints(
      Constants.SwerveConstants.maxAttainableSpeedMetersPerSecond, 
      Constants.SwerveConstants.maxAccelerationMetersPerSecondSquared));

  public DriveAuto(Rotation2d endAngle, Pose2d ...waypoints) {
    super(fromWaypoints(waypoints),
      () -> driveSubsystem.getOdometryLocation(),
      driveSubsystem.kinematics,
      xController, yController,
      thetaController,
      () -> endAngle,
      (states) -> driveSubsystem.setModuleStates(states),
      driveSubsystem);
  }

  public DriveAuto(double maxSpeedM, Rotation2d endAngle, Pose2d ...waypoints) {
    super(fromWaypoints(maxSpeedM, waypoints),
      () -> driveSubsystem.getOdometryLocation(),
      driveSubsystem.kinematics,
      xController, yController,
      thetaController,
      () -> endAngle,
      (states) -> driveSubsystem.setModuleStates(states),
      driveSubsystem);
  }
  
  public static Trajectory fromWaypoints(Pose2d ...waypoints) {
    return DriveAuto.fromWaypoints(Constants.SwerveConstants.maxAttainableSpeedMetersPerSecond, waypoints);
  }
  
  public static Trajectory fromWaypoints(double maxSpeedM, Pose2d ...waypoints) {
    TrajectoryConfig config = new TrajectoryConfig(
      maxSpeedM,
      Constants.SwerveConstants.maxAccelerationMetersPerSecondSquared);
    config.addConstraint(new SwerveDriveKinematicsConstraint(driveSubsystem.kinematics, maxSpeedM));
    
    return TrajectoryGenerator.generateTrajectory(List.of(waypoints), config);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveSubsystem.stop();
  }
}
