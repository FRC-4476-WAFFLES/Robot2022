// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  /** The array of swerve modules on the robot. */
  private final SwerveModule[] modules;

  /** Allows us to calculate the swerve module states from a chassis motion. */
  public final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final ADXRS450_Gyro ADXRS450Gyro = new ADXRS450_Gyro(Constants.gyroPort);
  private final AHRS ahrsIMU = new AHRS(SPI.Port.kMXP);

  private Translation2d goalOffsetFromStartingPosition = new Translation2d();

  public final Field2d field = new Field2d();

  public DriveSubsystem() {
    ArrayList<Translation2d> positions = new ArrayList<Translation2d>();
    ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();

    // Initialize each swerve module with its constants.
    for (SwerveConstants module : Constants.swerveModules) {
      modules.add(new SwerveModule(module));
      positions.add(module.position);
    }

    // Set up the kinematics and odometry.
    Translation2d[] positionArry = new Translation2d[positions.size()];
    for(int x = 0; x < positions.size(); x++){
      positionArry[x] = positions.get(x);
    }

    SwerveModule[] moduleArray = new SwerveModule[modules.size()];
    for (int x = 0; x < modules.size(); x++){
      moduleArray[x] = modules.get(x);
    }
    this.kinematics = new SwerveDriveKinematics(positionArry);
    this.modules = moduleArray;
    this.odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));
    
    ADXRS450Gyro.calibrate();

    SmartDashboard.putData("Field", field);
  }

  /** This method will be called once per scheduler run. */
  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for(int x=0; x<modules.length; x++){
      moduleStates[x] = modules[x].getState();
    }
    odometry.update(Rotation2d.fromDegrees(-ahrsIMU.getAngle()), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
    SmartDashboard.putNumber("X location", getOdometryLocation().getX());
    SmartDashboard.putNumber("Y location", getOdometryLocation().getY());
    SmartDashboard.putNumber("Odometry Heading", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro Heading", -ADXRS450Gyro.getAngle());
    SmartDashboard.putNumber("Gyro Rate", -ADXRS450Gyro.getRate());
    SmartDashboard.putNumber("New Gyro Angle", ahrsIMU.getAngle());
    SmartDashboard.putNumber("New Gyro Rate", ahrsIMU.getRate());
    SmartDashboard.putNumber("Vx", getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Vy", getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Vomega", getChassisSpeeds().omegaRadiansPerSecond);

    for (int x = 0; x < modules.length; x++) {
      SmartDashboard.putNumber("Module " + String.valueOf(x) + " Speed", moduleStates[x].speedMetersPerSecond);
    }

    int i = 1;
    for (SwerveModule module : modules) {
      SmartDashboard.putNumber("Encoder" + String.valueOf(i), module.getState().angle.getDegrees());
      i++;
    }

    if(Robot.isReal()) {
      field.setRobotPose(this.getOdometryLocation());
    }
    //field.setRobotPose(this.getOdometryLocation());

    SmartDashboard.putNumber("Drive Target Heading (Degrees)", Math.toDegrees(Math.atan2(-getOdometryLocation().getY(), getOdometryLocation().getX())));
  }

  public void robotDrive(double forward, double right, double rotation, boolean fieldCentric){
    ChassisSpeeds chassisSpeeds;

    double robotRotationRate = -ahrsIMU.getRate();
    robotRotationRate = (robotRotationRate / 180.0) * Math.PI;

    if (forward != 0 || right != 0) {
      rotation += robotRotationRate;
    }

    if (fieldCentric){
      //chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, gyro.getHeadingAsRotation2d());
      //chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, Rotation2d.fromDegrees(-ahrsIMU.getAngle()));
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, odometry.getPoseMeters().getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
    }
    
    SwerveModuleState[] swerveModuleState = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState, Constants.SwerveConstants.maxAttainableSpeedMetersPerSecond);
    setModuleStates(swerveModuleState);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates){
    for(int x=0; x<modules.length; x++){
      modules[x].drive(swerveModuleStates[x]);
    }
  }

  public Pose2d getOdometryLocation(){
    return new Pose2d(
      -odometry.getPoseMeters().getX(), 
      odometry.getPoseMeters().getY(), 
      odometry.getPoseMeters().getRotation());
  }

  public Pose2d getGoalOdometryLocation() {
    return new Pose2d(
      -odometry.getPoseMeters().getX() + goalOffsetFromStartingPosition.getX(), 
      odometry.getPoseMeters().getY() - goalOffsetFromStartingPosition.getY(), 
      odometry.getPoseMeters().getRotation()
    );
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for(int x=0; x<modules.length; x++){
      moduleStates[x] = modules[x].getState();
    }
    return kinematics.toChassisSpeeds(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
  }

  /** Stop all motors from running. */
  public void stop() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void resetGyro() {
    ahrsIMU.reset();
  }

  public void resetOdometry(Pose2d robotPose) {
    odometry.resetPosition(robotPose, Rotation2d.fromDegrees(-ahrsIMU.getAngle()));
  }

  public void setGoalOffset(Translation2d newGoalOffset) {
    this.goalOffsetFromStartingPosition = newGoalOffset;
  }

  public void resetSteerEncoders() {
    for (SwerveModule module : modules) {
      module.resetSteerEncoder();
    }
  }
}
