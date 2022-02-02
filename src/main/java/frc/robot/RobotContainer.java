// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.conveyor.ConveyorStop;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.ShooterDriverStationControl;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotIMU;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RobotIMU.GyroType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();

  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final RobotIMU IMU = new RobotIMU(GyroType.ADIS16448);
  public static final Camera vision = new Camera();

  private final DriveTeleop swerve = new DriveTeleop();
  private final IntakeStop intakeStop = new IntakeStop();
  private final ConveyorStop conveyorStop = new ConveyorStop();
  private final ShooterDriverStationControl shooterDriverStationControl = new ShooterDriverStationControl();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(swerve);
    intakeSubsystem.setDefaultCommand(intakeStop);
    conveyorSubsystem.setDefaultCommand(conveyorStop);
    shooterSubsystem.setDefaultCommand(shooterDriverStationControl);
    CommandScheduler.getInstance().registerSubsystem(IMU);
    CommandScheduler.getInstance().registerSubsystem(vision);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
