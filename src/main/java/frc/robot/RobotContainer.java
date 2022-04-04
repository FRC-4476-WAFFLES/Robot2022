// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.tools.Diagnostic;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.ThreeBallAutoPath;
import frc.robot.commands.autonomous.TwoBallAutoComplete;
import frc.robot.commands.autonomous.FenderHighShotComplete;
import frc.robot.commands.autonomous.FenderHighShotSetup;
import frc.robot.commands.autonomous.FiveBallAutoComplete;
import frc.robot.commands.autonomous.ResetToRightAutoStartingPosition;
import frc.robot.commands.autonomous.FiveBallAutoPath;
import frc.robot.commands.autonomous.ResetToLeftAutoStartingPosition;
import frc.robot.commands.autonomous.ThreeBallAutoComplete;
import frc.robot.commands.autonomous.TwoBallAutoPath;
import frc.robot.commands.autonomous.TwoBallExtendedAutoComplete;
import frc.robot.commands.climber.ClimberAnalogStickControl;
import frc.robot.commands.colourStuff.SetColourBasedOnRobotState;
import frc.robot.commands.drive.DriveCameraAim;
import frc.robot.commands.drive.DriveResetGyro;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.IntakeTeleop;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterDriverStationControl;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterVisionSetup;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.triggers.ShooterReadyTrigger;

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
  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final XboxController operate = new XboxController(2);
  public static final Camera vision = new Camera();

  public static final LightController lightController = new LightController();

  private final DriveTeleop swerve = new DriveTeleop();
  private final ShooterStop shooterStop = new ShooterStop();
  private final IntakeTeleop intakeTeleop = new IntakeTeleop();
  private final ClimberAnalogStickControl climberAnalogStickControl = new ClimberAnalogStickControl();
  private final SetColourBasedOnRobotState autoUpdateColour = new SetColourBasedOnRobotState();
  
  private final FenderHighShotComplete autoShot = new FenderHighShotComplete();
  private final TwoBallAutoPath twoBallAutoPath = new TwoBallAutoPath();
  private final ThreeBallAutoPath threeBallAutoPath = new ThreeBallAutoPath();
  private final FiveBallAutoPath fiveBallAutoPath = new FiveBallAutoPath();
  private final TwoBallAutoComplete twoBallAutoComplete = new TwoBallAutoComplete();
  private final ThreeBallAutoComplete threeBallAutoComplete = new ThreeBallAutoComplete();
  private final FiveBallAutoComplete fiveBallAutoComplete = new FiveBallAutoComplete();
  private final ResetToRightAutoStartingPosition resetToRightFender = new ResetToRightAutoStartingPosition();
  private final ResetToLeftAutoStartingPosition resetToLeftFender = new ResetToLeftAutoStartingPosition();
  private final TwoBallExtendedAutoComplete twoBallExtendedAutoComplete = new TwoBallExtendedAutoComplete();
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(swerve);
    intakeSubsystem.setDefaultCommand(intakeTeleop);
    conveyorSubsystem.setDefaultCommand(intakeTeleop);
    shooterSubsystem.setDefaultCommand(shooterStop);
    climberSubsystem.setDefaultCommand(climberAnalogStickControl);
    lightController.setDefaultCommand(autoUpdateColour);
    CommandScheduler.getInstance().registerSubsystem(vision);

    // Configure the button bindings
    configureButtonBindings();

    // Add the auto modes
    autoChooser.addOption("Auto Shot", autoShot);
    autoChooser.addOption("2 Ball Auto Path", twoBallAutoPath);
    autoChooser.addOption("3 Ball Auto Path", threeBallAutoPath);
    autoChooser.addOption("5 Ball Auto Path", fiveBallAutoPath);
    autoChooser.addOption("2 Ball Auto Complete", twoBallAutoComplete);
    autoChooser.addOption("3 Ball Auto Complete", threeBallAutoComplete);
    autoChooser.addOption("5 Ball Auto Complete", fiveBallAutoComplete);
    autoChooser.addOption("Reset Right Fender", resetToRightFender);
    autoChooser.addOption("Reset Left Fender", resetToLeftFender);
    autoChooser.addOption("2 Ball Extended Auto Complete", twoBallExtendedAutoComplete);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    vision.setProcesingMode(Camera.ProcessingMode.Vision);
    vision.setPipeline(Camera.Pipeline.TheOnlyPipelineWeAreUsing);
    vision.setLEDMode(Camera.CameraLEDMode.Off);
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

    //final var leftButton = new JoystickButton(operate, XboxController.Button.kLeftBumper.value);
    //final var rightButton = new JoystickButton(operate, XboxController.Button.kRightBumper.value);

    final var back = new JoystickButton(operate, XboxController.Button.kBack.value);

    final var povUp = new POVButton(operate, 0);
    //final var povRight = new POVButton(operate, 90);
    final var povDown = new POVButton(operate, 180);

    final var right1 = new JoystickButton(rightJoystick, 1);
    final var right7 = new JoystickButton(rightJoystick, 7);
    final var right10 = new JoystickButton(rightJoystick, 10);

    final Trigger[] aimOverrideTriggers = new Trigger[] {
      new JoystickButton(rightJoystick, 2),
      new JoystickButton(rightJoystick, 3),
      new JoystickButton(rightJoystick, 4),
      new JoystickButton(rightJoystick, 5),
    };

    final var shooterReadyTrigger = new ShooterReadyTrigger();
/*
    a.whileHeld(new Intake2Balls());
    b.whileHeld(new IntakeOut().alongWith(new ConveyorOut()));
    x.whenPressed(new IntakeDeploy());
    y.whenPressed(new IntakeRetract());*/

    //a.whenPressed(new IntakeDeploy());
    //b.whenPressed(new IntakeRetract());
    a.whenPressed(new InstantCommand(climberSubsystem::nextSetpoint));
    b.whenPressed(new InstantCommand(climberSubsystem::previousSetpoint));

    povUp.whileActiveContinuous(new ShooterVisionSetup().perpetually());
    povDown.whileActiveContinuous(new FenderHighShotSetup().perpetually());
    
    x.and(shooterReadyTrigger.or(y)).whileActiveContinuous(new Shoot().perpetually());
    //x.whileActiveContinuous(new Shoot().perpetually());
    back.toggleWhenPressed(new ShooterDriverStationControl());

    //rightJoystickButton3.whileHeld(new DriveCameraAim()).or(povUp).toggleWhenActive(new ShooterVisionSetup().perpetually());

    right1.whileHeld(new DriveCameraAim(aimOverrideTriggers).perpetually());
    
    right7.and(right10).debounce(0.1).whenActive(new DriveResetGyro().alongWith(new InstantCommand(driveSubsystem::resetSteerEncoders)));
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
  