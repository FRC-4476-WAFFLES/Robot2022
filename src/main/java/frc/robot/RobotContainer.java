// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.conveyor.ConveyorIn;
import frc.robot.commands.conveyor.ConveyorOut;
import frc.robot.commands.conveyor.ConveyorStop;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.IntakeDeploy;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.IntakeRetract;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.ShooterKickerWheelSpinup;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWheelSpinup;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final XboxController operate = new XboxController(2);
  public static final Camera vision = new Camera();

  private final DriveTeleop swerve = new DriveTeleop();
  private final IntakeStop intakeStop = new IntakeStop();
  private final ConveyorStop conveyorStop = new ConveyorStop();
  private final ShooterStop shooterStop = new ShooterStop();
  
  private final DriveAuto testAuto = new DriveAuto(0.1,
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(Units.feetToMeters(5), 0, new Rotation2d(0))
  );
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(swerve);
    intakeSubsystem.setDefaultCommand(intakeStop);
    conveyorSubsystem.setDefaultCommand(conveyorStop);
    shooterSubsystem.setDefaultCommand(shooterStop);
    CommandScheduler.getInstance().registerSubsystem(vision);

    // Configure the button bindings
    configureButtonBindings();

    // Add the auto modes
    autoChooser.addOption("Test Auto", testAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final var a = new JoystickButton(operate, XboxController.Button.kA.value);
    final var b = new JoystickButton(operate, XboxController.Button.kB.value);
    final var x = new JoystickButton(operate, XboxController.Button.kX.value);
    final var y = new JoystickButton(operate, XboxController.Button.kY.value);

    final var leftButton = new JoystickButton(operate, XboxController.Button.kLeftBumper.value);
    final var rightButton = new JoystickButton(operate, XboxController.Button.kRightBumper.value);

    final var back = new JoystickButton(operate, XboxController.Button.kBack.value);

    a.whileHeld(new IntakeIn().alongWith(new ConveyorIn()));
    b.whileHeld(new IntakeOut().alongWith(new ConveyorOut()));
    x.whenPressed(new IntakeDeploy());
    y.whenPressed(new IntakeRetract());

    leftButton.whenPressed(new ShooterKickerWheelSpinup());
    rightButton.whenPressed(new ShooterWheelSpinup());
    back.whenPressed(new ShooterStop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
  