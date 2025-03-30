// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (90) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = 4;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    // Triger deadband
    public static final double TRIGGER_DEADBAND = 0.5;
  }

  public static final class ElevatorConstants{
    
    public static final int leftElevatorID = 21;
    public static final int limitSwitchPort = 0;
    // public static final int toplimitSwitchPort = 1;
    public static final int StickmotorID = 20;

   //countsper inch = (360 * gear ratio)/(diamiter of sprocket * pi) 
    public static final double countsPerInch = (360 * 10 )/(2* 3.14159265359);
    
    public static final double downPos = .1;
    public static final double L1 = 10;
    public static final double L2 = 15.4;
    public static final double L3 = 42.6;
    public static final double L4 = 20;
    public static final double bottomPos = 0;
    
    public static final double minPos = 0;
    public static final double maxPos = 48;
    public static final double posTolerance = .25;

    public static final double maxVelocity = 0.01;
    public static final double maxAcceleration = 0.01;

    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final int kS = 0;
    public static final double kG = 0.036;
    public static final double kV = 1;

    public static final double max_output = 5;
  
  }

  public static final class Shooter{
    public static final int LeftMotorId = 61;

    public static final int ForBeamID = 3;
    public static final int BackBeamID = 4;

    public static final double IntakeSpeed = -0.12;
    public static final double ReverseSpeed = 0.2;
    public static final double StickSpeed = 1;
    public static final double SHOOTspeed = -.75;
    public static final double stickautospeed= 0.25;

    public static final double IntakeControledspeed = -100;
    public static final double ReverseControledspeed = 100;
    public static final double StickControledspeed = 100;
    public static final double SHOOTControledspeed = -1000;
    
    
    public static final double IndexSpeed = 20;
    public static final double L1Speed = .5;
    public static final double L24Speed = .5;

  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 10; // 25:1
    public static final double kCarriageMass = 10 * 0.453592; // lbs * 0.453592 = kg
    public static final double kElevatorDrumRadius = .75/39.3701; // in/39.3701 = m
    public static final double kMinElevatorHeightMeters = 0/39.3701; // in/39.3701 = m
    public static final double kMaxElevatorHeightMeters = 45.04/39.3701; // in/39.3701 = m
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationPID = new PIDConstants(6.0,0,0);
    public static final PIDConstants kRotationPID = new PIDConstants(5.0,0,0);

    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
        AutoConstants.kTranslationPID, 
        AutoConstants.kRotationPID
    );

    public enum PathplannerConfigs{
        PROGRAMMER_CHASSIS(new RobotConfig(
            Kilogram.of(10), 
            KilogramSquareMeters.of(1.9387211145),
            new ModuleConfig(
                Inches.of(3.75/2.0),
                MetersPerSecond.of(4),
                1.00,
                DCMotor.getNEO(1),
                6.75,
                Amps.of(40),
                1
            ),
            new Translation2d(Inches.of(12.625), Inches.of(12.5625)),
            new Translation2d(Inches.of(12.125), Inches.of(-12.5)),
            new Translation2d(Inches.of(-12), Inches.of(12.5)),
            new Translation2d(Inches.of(-12.125), Inches.of(-12.4375))
        )),
        COMP_CHASSIS(new RobotConfig(
            Kilogram.of(142), 
            KilogramSquareMeters.of(4.86247863),
            new ModuleConfig(
                Inches.of(3.75/2.0),
                MetersPerSecond.of(5.273),
                0.8,
                DCMotor.getNEO(1),
                5.900,
                Amps.of(40),
                1
            ),
            new Translation2d(Inches.of(13.5), Inches.of(11.5)),
            new Translation2d(Inches.of(13.625), Inches.of(-11.625)),
            new Translation2d(Inches.of(-13.625), Inches.of(11.5)),
            new Translation2d(Inches.of(-13.5), Inches.of(-10.4375))
    ));
;

        public RobotConfig config;

        private PathplannerConfigs(RobotConfig config) {
            this.config = config;
        }
    }

    public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
        new PIDConstants(5.5, 0.0, 0.1, 0.0), 
        AutoConstants.kRotationPID
    );

    public static final Time kAutoAlignPredict = Seconds.of(0.0);

    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(3.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.5);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(2);

    public static final Time kEndTriggerDebounce = Seconds.of(0.04);

    public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);


    public static final LinearVelocity kStationApproachSpeed = InchesPerSecond.of(8);
    public static final Time kStationApproachTimeout = Seconds.of(5);

    public static final PathConstraints kStartingPathConstraints = new PathConstraints(3.0, 2.0, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.

    public static final PathConstraints kTeleopPathConstraints = new PathConstraints(2.0, 1.75, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.

    public static final PathConstraints kAutoPathConstraints = new PathConstraints(2.0, 2.0, 1/2 * Math.PI, 1 * Math.PI); //? consider making these more aggressive
    
    public static final double kTriggerDistance = 2.5;

    public static final class StationVisualizationConstants {
            public static final Pose2d kBlueLeft = new Pose2d(0.947, 7.447, Rotation2d.fromDegrees(-50));
            public static final Pose2d kBlueRight = new Pose2d(0.947, 0.614, Rotation2d.fromDegrees(50));
            public static final Pose2d kRedLeft = new Pose2d(16.603, 0.614, Rotation2d.fromDegrees(130));
            public static final Pose2d kRedRight = new Pose2d(16.603, 7.447, Rotation2d.fromDegrees(-120));
    }
  }


}
