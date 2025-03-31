// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ScoringSystem;
import frc.robot.subsystems.Elevator.Setpoint;
import frc.robot.subsystems.TargetingSystem.ReefBranchLevel;
import frc.robot.subsystems.TargetingSystem.ReefBranchSide;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import javax.swing.text.Position;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.TargetingSystem;
import frc.robot.subsystems.ScoringSystem;
import frc.robot.subsystems.swervedrive.PositionPIDCommand;
import frc.robot.subsystems.swervedrive.PositionPIDCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  // Intialise the elevator subsystem
  private final Elevator    elevatorsub           = new Elevator();
  private final TargetingSystem targetingSystem = new TargetingSystem();
  private final ScoringSystem scoringSystem = new ScoringSystem(elevatorsub, drivebase, targetingSystem, null);
  private final PositionPIDCommand positionPIDCommand = new PositionPIDCommand(drivebase, targetingSystem.getCoralTargetPose());
  

  // private final SendableChooser<Command> autoChooser;
  // private final SendableChooser<Command> autoChooser;

  


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() *-1,
                                                                () -> driverXbox.getLeftX() *-1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -driverXbox.getRightX(),
                                                                                             () -> -driverXbox.getRightY())
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    // NamedCommands.registerCommand("drive in auto", drivebase.driveForAuton());
    NamedCommands.registerCommand("L2", elevatorsub.setSetpointCommandPP(Setpoint.kLevel2).repeatedly().until(elevatorsub.L2Trigger()));
    NamedCommands.registerCommand("L3", elevatorsub.setSetpointCommandPP(Setpoint.kLevel3).repeatedly().until(elevatorsub.L3Trigger()));
    NamedCommands.registerCommand("Shoot", elevatorsub.SHOOTCommand().until(elevatorsub.NOTHoldingCoralTrigger()));
    NamedCommands.registerCommand("L0", elevatorsub.setSetpointCommand(Setpoint.kFeederStation).repeatedly().until(elevatorsub.L0Trigger()));
    NamedCommands.registerCommand("dealgea", elevatorsub.DeAlgea());
    NamedCommands.registerCommand("Intake", elevatorsub.runIntakeCommand().until(elevatorsub.CoralReadyTrigger()));
    NamedCommands.registerCommand("LockDrive", Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    NamedCommands.registerCommand("AlignToLeft", 
    targetingSystem.setBranchLevel(ReefBranchLevel.L2).andThen(targetingSystem.setBranchSide(ReefBranchSide.LEFT))
    .andThen(targetingSystem.autoTargetCommand(drivebase::getPose))
    .andThen(Commands.runOnce(() -> drivebase.getSwerveDrive().field.getObject("target")
    .setPose(targetingSystem.getCoralTargetPose())))
    .andThen(targetingSystem.driveToCoralTargetDefer(drivebase)));
    NamedCommands.registerCommand("AlignToRight", 
    targetingSystem.setBranchLevel(ReefBranchLevel.L2).andThen(targetingSystem.setBranchSide(ReefBranchSide.LEFT))
    .andThen(targetingSystem.autoTargetCommand(drivebase::getPose))
    .andThen(Commands.runOnce(() -> drivebase.getSwerveDrive().field.getObject("target")
    .setPose(targetingSystem.getCoralTargetPose())))
    .andThen(targetingSystem.driveToCoralTargetDefer(drivebase)));

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(4, 4)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                                () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // Start button -> Zero gyro
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // D pad up -> Set elevator to feeder station height
      driverXbox.povUp().onTrue(elevatorsub.setSetpointCommand(Setpoint.kFeederStation));
      // D pad right -> Set elevator to level 1 height
      driverXbox.povRight().onTrue(elevatorsub.setSetpointCommand(Setpoint.kLevel1));
      // D pad down -> Set elevator to level 2 height
      driverXbox.povDown().onTrue(elevatorsub.setSetpointCommand(Setpoint.kLevel2));
      // D pad left -> Set elevator to level 3 height
      driverXbox.povLeft().onTrue(elevatorsub.setSetpointCommand(Setpoint.kLevel3));
      // Right Trigger -> Run intake to apropiate speed or intake depending on elevator height
      driverXbox.rightTrigger(OperatorConstants.TRIGGER_DEADBAND).whileTrue(elevatorsub.runIntakeCommand().until(elevatorsub.CoralReadyTrigger()));
      // Left Trigger -> Run intake in reverse
      driverXbox.leftTrigger(OperatorConstants.TRIGGER_DEADBAND).whileTrue(elevatorsub.reverseIntakeCommand());
      // Right Bumper -> force intake forwards
      driverXbox.rightBumper().whileTrue(elevatorsub.SHOOTCommand());
      // Left Bumper -> Run stick
      driverXbox.leftBumper().whileTrue(elevatorsub.runstickCommand());

      driverXbox.a().whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L2).andThen(targetingSystem.setBranchSide(ReefBranchSide.LEFT))
                                                      .andThen(targetingSystem.autoTargetCommand(drivebase::getPose))
                                                      .andThen(Commands.runOnce(() -> drivebase.getSwerveDrive().field.getObject(
                                                          "target").setPose(targetingSystem.getCoralTargetPose())))
                                                      .andThen(targetingSystem.driveToCoralTargetDefer(drivebase)));
      driverXbox.b().whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L2).andThen(targetingSystem.setBranchSide(ReefBranchSide.RIGHT))
                                                      .andThen(targetingSystem.autoTargetCommand(drivebase::getPose))
                                                      .andThen(Commands.runOnce(() -> drivebase.getSwerveDrive().field.getObject(
                                                          "target").setPose(targetingSystem.getCoralTargetPose())))
                                                      .andThen(targetingSystem.driveToCoralTargetDefer(drivebase)));
    
    driverXbox.x().whileTrue(positionPIDCommand);
      
      // move elevator zero down
      // driverXbox.a().onTrue(Commands.runOnce(elevatorsub::downelevatorCommand));
      
      // release stick
      // driverXbox.b().whileTrue(Commands.runOnce(elevatorsub::sendstick));
      // driverXbox.b().onFalse(Commands.runOnce(elevatorsub::downstick));

      // driverXbox.x().whileTrue(targetingSystem.setBranchSide(ReefBranchSide.LEFT));
      // driverXbox.y().whileTrue(targetingSystem.setBranchSide(ReefBranchSide.RIGHT));

      //score coral
      // driverXbox.b().whileTrue(scoringSystem.scoreCoral());


    }
  
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("Left 16 Point auto");
    // return drivebase.getAutonomousCommand("Intake shoot test");
    // return autoChooser.getSelected();
    return drivebase.getAutonomousCommand("Right 16 point auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
