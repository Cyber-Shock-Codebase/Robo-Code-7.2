package frc.robot.subsystems;

import static java.lang.Math.abs;

import java.util.concurrent.locks.Condition;

import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Shooter;
import frc.robot.Configs;
import frc.robot.Constants.SimulationRobotConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SubsystemBase {
  
    /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

//   // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
//   // initialize the closed loop controller and encoder.
//   private SparkMax armMotor =
//       new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
//   private SparkClosedLoopController armController = armMotor.getClosedLoopController();
//   private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  // Initialize limit switches for the elevator
  private DigitalInput BottomLimitSwitch = new DigitalInput(ElevatorConstants.limitSwitchPort);
  // private DigitalInput TopLimitSwitch = new DigitalInput(ElevatorConstants.toplimitSwitchPort);

  // Initialize intake sensors
    private DigitalInput forbeam = new DigitalInput(Shooter.ForBeamID);
    private DigitalInput backbeam = new DigitalInput(Shooter.BackBeamID);

  // Initialize intake SPARK. We will use a closed loop controller like above.
  private SparkMax intakeMotor = new SparkMax(Shooter.LeftMotorId, MotorType.kBrushless);
  private SparkClosedLoopController IntakeClosedLoopController = intakeMotor.getClosedLoopController();
  private RelativeEncoder IntakeEncoder = intakeMotor.getEncoder();
  
  // Initialize stick SPARK. We will a closed loop controller like above.
  private SparkMax StickMotor = new SparkMax(ElevatorConstants.StickmotorID, MotorType.kBrushless);
  private SparkClosedLoopController StickClosedLoopController = StickMotor.getClosedLoopController();
  private RelativeEncoder StickEncoder = StickMotor.getEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private boolean wasResetByOffsetTopLimit = false;
//   private double armCurrentTarget = ArmSetpoints.kFeederStation;
  private double elevatorCurrentTarget = ElevatorConstants.downPos;

  private Timer limitSwitchTimer;

  // Simulation setup and variables
  private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);
  Servo exampleServo = new Servo(1);
  // private DCMotor armMotorModel = DCMotor.getNEO(1);
  // private SparkMaxSim armMotorSim;
//   private final SingleJointedArmSim m_armSim =
//       new SingleJointedArmSim(
//           armMotorModel,
//           SimulationRobotConstants.kArmReduction,
//           SingleJointedArmSim.estimateMOI(
//               SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
//           SimulationRobotConstants.kArmLength,
//           SimulationRobotConstants.kMinAngleRads,
//           SimulationRobotConstants.kMaxAngleRads,
//           true,
//           SimulationRobotConstants.kMinAngleRads,
//           0.0,
//           0.0);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));
//   private final MechanismLigament2d m_armMech2d =
//       m_elevatorMech2d.append(
//           new MechanismLigament2d(
//               "Arm",
//               SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
//               180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));
public Elevator() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    // armMotor.configure(
    //     Configs.CoralSubsystem.armConfig,
    //     ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);
    elevatorMotor.configure(
        Configs.CoralSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.configure(
        Configs.CoralSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    StickMotor.configure(
        Configs.CoralSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  

    // Display mechanism2d
    SmartDashboard.putData("Coral Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    // armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);

    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
    // armMotorSim = new SparkMaxSim(armMotor, armMotorModel);

    limitSwitchTimer = new Timer();
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    // armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    // if Coral is out of the way or the elevator is not at the down position, move the elevator
    if(!isCoralproblematic() || elevatorCurrentTarget != ElevatorConstants.downPos) {
      elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }else {
      elevatorMotor.set(0);
    }
    
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && BottomLimitSwitch.get()) {
      // try {
      //   wait(750);
      // } catch (InterruptedException e) {
      //   // TODO Auto-generated catch block
      //   System.out.println("wait method has an error");
      //   e.printStackTrace();
      // }
      limitSwitchTimer.reset();
      limitSwitchTimer.start();
      System.out.println("limit timer started");


      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      
      
      wasResetByLimit = true;
    } else if (!BottomLimitSwitch.get()) {
      wasResetByLimit = false;
    }

    if (BottomLimitSwitch.get() && limitSwitchTimer.isRunning() && limitSwitchTimer.get() > 1) {
      elevatorEncoder.setPosition(ElevatorConstants.minPos);
      System.out.println("bottomlimit");
      limitSwitchTimer.reset();
      limitSwitchTimer.stop();
    }
  }

  // private void offsetElevatorOnTopLimitSwitch() {
  //   if (!wasResetByOffsetTopLimit && TopLimitSwitch.get()) {
  //     // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
  //     // prevent constant zeroing while pressed
  //     elevatorEncoder.setPosition(ElevatorConstants.maxPos);
  //     wasResetByOffsetTopLimit = true;
  //   } else if (!TopLimitSwitch.get()) {
  //       wasResetByOffsetTopLimit = false;
  //   }
  // }

  public boolean isCoralproblematic() {
    return !backbeam.get();
  }

  public boolean isHoldingCoral() {
    return !forbeam.get();
  }

  public boolean isCoralReady() {
    return isHoldingCoral() && !isCoralproblematic();
  }

  // public boolean iselevatorfree() {
  //   return !TopLimitSwitch.get() && !BottomLimitSwitch.get();
  // }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
    //   armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  private void setStickpower(double power) {
     StickMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
            //   armCurrentTarget = ArmSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorConstants.downPos;
              break;
            case kLevel1:
            //   armCurrentTarget = ArmSetpoints.kLevel1;
              elevatorCurrentTarget = ElevatorConstants.L1;
              break;
            case kLevel2:
            //   armCurrentTarget = ArmSetpoints.kLevel2;
              elevatorCurrentTarget = ElevatorConstants.L2;
              break;
            case kLevel3:
            //   armCurrentTarget = ArmSetpoints.kLevel3;
              elevatorCurrentTarget = ElevatorConstants.L3;
              break;
            case kLevel4:
            //   armCurrentTarget = ArmSetpoints.kLevel4;
              elevatorCurrentTarget = ElevatorConstants.L4;
              break;
          }
        });
  }

  public Command setSetpointCommandPP(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
            //   armCurrentTarget = ArmSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorConstants.downPos;
              if(ElevatorConstants.posTolerance >= abs(ElevatorConstants.downPos - elevatorEncoder.getPosition())){
                break;
              }
              break;
            case kLevel1:
            //   armCurrentTarget = ArmSetpoints.kLevel1;
              elevatorCurrentTarget = ElevatorConstants.L1;
              if(ElevatorConstants.posTolerance >= abs(ElevatorConstants.L1 - elevatorEncoder.getPosition())){
                break;
              }
              break;
            case kLevel2:
            //   armCurrentTarget = ArmSetpoints.kLevel2;
              elevatorCurrentTarget = ElevatorConstants.L2;
              if(ElevatorConstants.posTolerance >= abs(ElevatorConstants.L2 - elevatorEncoder.getPosition())){
                break;
              }
            case kLevel3:
            //   armCurrentTarget = ArmSetpoints.kLevel3;
              elevatorCurrentTarget = ElevatorConstants.L3;
              if(ElevatorConstants.posTolerance >= abs(ElevatorConstants.L3 - elevatorEncoder.getPosition())){
                break;
              }
              break;
            case kLevel4:
            //   armCurrentTarget = ArmSetpoints.kLevel4;
              elevatorCurrentTarget = ElevatorConstants.L4;
              if(ElevatorConstants.posTolerance >= abs(ElevatorConstants.L4 - elevatorEncoder.getPosition())){
                break;
              }
              break;
          }
        });
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released or coral is in position,
   * the motor will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(Shooter.IntakeSpeed), () -> this.setIntakePower(0.0));
        // () -> IntakeClosedLoopController.setReference(Shooter.IntakeControledspeed, ControlType.kMAXMotionVelocityControl),
        // () -> IntakeClosedLoopController.setReference(Shooter.IntakeControledspeed, ControlType.kMAXMotionVelocityControl));
  }

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(Shooter.ReverseSpeed), () -> this.setIntakePower(0.0));
        // () -> IntakeClosedLoopController.setReference(Shooter.ReverseControledspeed, ControlType.kMAXMotionVelocityControl),
        // () -> IntakeClosedLoopController.setReference(0.0, ControlType.kMAXMotionVelocityControl));

  }

  public Command SHOOTCommand() {
    return this.startEnd(
        () -> this.setIntakePower(Shooter.SHOOTspeed), () -> this.setIntakePower(0.0));
        // () -> IntakeClosedLoopController.setReference(Shooter.SHOOTControledspeed, ControlType.kMAXMotionVelocityControl),
        // () -> IntakeClosedLoopController.setReference(0.0, ControlType.kMAXMotionVelocityControl));
  }

  public Command runstickCommand() {
    return this.startEnd(
        () -> this.setStickpower(Shooter.StickSpeed), () -> this.setStickpower(0.0));
        // () -> StickClosedLoopController.setReference(Shooter.StickControledspeed, ControlType.kMAXMotionVelocityControl),
        // () -> StickClosedLoopController.setReference(0.0, ControlType.kMAXMotionVelocityControl));
  }
  

  public Command runstickCommandDONTSTOP() {
    return this.run(
        () -> this.setStickpower(Shooter.stickautospeed));
        // () -> StickClosedLoopController.setReference(Shooter.StickControledspeed, ControlType.kMAXMotionVelocityControl),
        // () -> StickClosedLoopController.setReference(0.0, ControlType.kMAXMotionVelocityControl));
  }

  public void downelevatorCommand() {
    elevatorEncoder.setPosition(elevatorEncoder.getPosition() + 5);
        // () -> IntakeClosedLoopController.setReference(Shooter.ReverseControledspeed, ControlType.kMAXMotionVelocityControl),
        // () -> IntakeClosedLoopController.setReference(0.0, ControlType.kMAXMotionVelocityControl));

  }

  public void sendstick(){
    exampleServo.set(0);
  }

  public void downstick(){
    exampleServo.set(1);
  }
  
  public Trigger CoralReadyTrigger() {
    return new Trigger(this::isCoralReady);
  }

  public Trigger HoldingCoralTrigger() {
    return new Trigger(this::isHoldingCoral);
  }

  public Trigger NOTHoldingCoralTrigger() {
    return new Trigger(() -> !isHoldingCoral());
  }

  public Trigger L0Trigger() {
    return new Trigger(() -> ElevatorConstants.posTolerance >= abs(ElevatorConstants.downPos - elevatorEncoder.getPosition()));
  }

  public Trigger L1Trigger() {
    return new Trigger(() -> ElevatorConstants.posTolerance >= abs(ElevatorConstants.L1 - elevatorEncoder.getPosition()));
  }

  public Trigger L2Trigger() {
    return new Trigger(() -> ElevatorConstants.posTolerance >= abs(ElevatorConstants.L2 - elevatorEncoder.getPosition()));
  }

  public Trigger L3Trigger() {
    return new Trigger(() -> ElevatorConstants.posTolerance >= abs(ElevatorConstants.L3 - elevatorEncoder.getPosition()));
  }

  public Command DeAlgea() {
    return this.runOnce(
        () -> {
          if(isCoralReady()){
            elevatorCurrentTarget = ElevatorConstants.L2;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();
    // offsetElevatorOnTopLimitSwitch();

    // Display subsystem values
    // SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    // SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putString("BottomLimitSwitch", BottomLimitSwitch.get()? "Pressed" : "Not Pressed");
    // SmartDashboard.putString("TopLimitSwitch", TopLimitSwitch.get()? "Pressed" : "Not Pressed");
    SmartDashboard.putBoolean("ForBeam Triped", !forbeam.get());
    SmartDashboard.putBoolean("BackBeam Triped", !backbeam.get());
    SmartDashboard.putBoolean("Coral Problematic", isCoralproblematic());
    SmartDashboard.putBoolean("Holding coral", isHoldingCoral());
    SmartDashboard.putBoolean("Coral ready", isCoralReady());

    // Update mechanism2d
    m_elevatorMech2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    // m_armMech2d.setAngle(
    //     180
    //         - ( // mirror the angles so they display in the correct direction
    //         Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
    //             + Units.rotationsToDegrees(
    //                 armEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
    //         - 90 // subtract 90 degrees to account for the elevator
    //     );
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    // m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    // m_armSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    // armMotorSim.iterate(
    //     Units.radiansPerSecondToRotationsPerMinute(
    //         m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
    //     RobotController.getBatteryVoltage(),
    //     0.02);

    // SimBattery is updated in Robot.java
  }
}