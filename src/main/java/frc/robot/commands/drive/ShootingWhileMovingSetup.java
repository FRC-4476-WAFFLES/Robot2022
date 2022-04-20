// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Camera.CameraLEDMode;

public class ShootingWhileMovingSetup extends DriveAndPIDTurn {
  static Supplier<Rotation2d> targetHeading = driveSubsystem.getOdometryLocation()::getRotation;
  static final PIDController turnController = new PIDController(-5.5, 0, -0.4); // -5.0, 0, -0.2

  private boolean isActive = false;

  /** Creates a new ShootingWhileMovingSetup. */
  public ShootingWhileMovingSetup() {
    super(
      targetHeading, 
      turnController, 
      SwerveConstants.aimToleranceDegrees, 
      vision, 
      shooterSubsystem, 
      driveSubsystem
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setLEDMode(CameraLEDMode.On);
    super.initialize();
    isActive = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isActive = true;

    double distanceToGoal = vision.getHorisontalFieldDistanceToGoal(); // Get and store distance to goal
    Rotation2d angleToGoal = Rotation2d.fromDegrees(vision.getHorizontal()); // Get and store angle between shooter and goal

    double v_shooter = shooterSubsystem.rpmToMetersPerSecond(shooterSubsystem.calculateShooterTargetSpeed(distanceToGoal)); // Calculate shooter speed based on distance, assuming robot stationary
    double theta_shooter = shooterSubsystem.rotationsToHoodAngleDegrees(shooterSubsystem.calculateShooterTargetAngle(distanceToGoal)); // Calculate shooter angle based on distance, assuming robot stationary

    ChassisSpeeds v_chassis = driveSubsystem.getChassisSpeeds(); // Get and store robot velocity, then convert to velocity towards and tangent to the hub
    double v_radial = v_chassis.vxMetersPerSecond * angleToGoal.getCos() + v_chassis.vyMetersPerSecond * angleToGoal.getSin();
    double v_tangential = -v_chassis.vxMetersPerSecond * angleToGoal.getSin() + v_chassis.vyMetersPerSecond * angleToGoal.getCos();

    double v_shooter_x = v_shooter * Math.sin(Math.toRadians(theta_shooter)); // Calculate horisontal velocity of shooter
    
    double tangentialDistanceOfBall = (v_tangential / v_shooter_x) * distanceToGoal; // Calculate how far ball will travel tangentially to hub by taking the ratio of the shooter horisontal velocity and the tangential velocity of the robot, then multiplying by the distance to the goal

    double distanceToCompensatedGoal = Math.sqrt(Math.pow(distanceToGoal, 2) + Math.pow(tangentialDistanceOfBall, 2)); // Distance that the ball will have to travel to get the the goal given the robot's current tangential velocity

    double v_shooter_intermediate = shooterSubsystem.rpmToMetersPerSecond(shooterSubsystem.calculateShooterTargetSpeed(distanceToCompensatedGoal)); // Calculate shooter speed based on compensated distance
    double theta_shooter_intermediate = shooterSubsystem.rotationsToHoodAngleDegrees(shooterSubsystem.calculateShooterTargetAngle(distanceToCompensatedGoal)); // Calculate shooter angle based on compensated distance
    double omega_shooter = Math.toDegrees(Math.atan2(tangentialDistanceOfBall, distanceToGoal)); // Calculate the angle the robot has to face to score the ball

    double v_shooter_new_h = v_shooter_intermediate * Math.cos(Math.toRadians(theta_shooter_intermediate)); // Calculate shooter vertical velocity
    double v_shooter_intermediate_horisontal = v_shooter_intermediate * Math.sin(Math.toRadians(theta_shooter_intermediate)); // Calculate shooter horisontal velocity
    double v_shooter_intermediate_x = v_shooter_intermediate_horisontal * Math.cos(Math.toRadians(omega_shooter)); // Calculate shooter x velocity
    double v_shooter_new_y = v_shooter_intermediate_horisontal * Math.sin(Math.toRadians(omega_shooter)); // Calculate shooter y velocity

    double v_shooter_new_x = v_shooter_intermediate_x - v_radial; // Subtract robot radial velocity from shooter x velocity

    double v_shooter_new = Math.sqrt(Math.pow(v_shooter_new_x, 2) + Math.pow(v_shooter_new_y, 2) + Math.pow(v_shooter_new_h, 2)); // Calculate shooter full velocity
    double theta_shooter_new = Math.toDegrees(Math.atan2(v_shooter_new_x, v_shooter_new_h)); // Calculate shooter angle
    double omega_shooter_new = Math.atan2(v_shooter_new_y, v_shooter_new_x); // Calculate robot angle (in radians)

    targetHeading = () -> driveSubsystem.getOdometryLocation().getRotation().minus(new Rotation2d(omega_shooter_new));
    shooterSubsystem.setHoodAngle(shooterSubsystem.hoodAngleDegreesToRotations(theta_shooter_new));
    shooterSubsystem.setShooterSpeed(shooterSubsystem.metersPerSecondToRPM(v_shooter_new));
    shooterSubsystem.setKickerSpeed(0.8);

    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    vision.setLEDMode(CameraLEDMode.Off);
    shooterSubsystem.stop();
    isActive = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Trigger getIsActiveTrigger() {
    return new Trigger(() -> isActive);
  }

  public Trigger getIsReadyTrigger() {
    return new Trigger(() -> super.isFinished() && vision.getHasTarget());
  }

  // Unused code that could be good to keep for reference:

  /*
  double calcTangentialDistanceOfBall = v_tangential * timeOfFlight;
  double calcRadialDistanceOfBall = v_shooter_x * timeOfFlight;
  double ratio = distance / calcRadialDistanceOfBall;
  return calcTangentialDistanceOfBall * ratio;*/
  
  /*
  double timeToApex = v_shooter_h / 9.81;
  double apexHeight = v_shooter_h * timeToApex / 2;
  double timeFromApex = Math.sqrt((apexHeight - ShooterConstants.goalHeightAboveShooter) / 4.905);
  double timeOfFlight = timeToApex + timeFromApex;*/
}
