package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {

  public static final class CoralSubsystem {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig StickConfig = new SparkMaxConfig();

    static {
      // // Configure basic settings of the arm motor
      // armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

      // /*
      //  * Configure the closed loop controller. We want to make sure we set the
      //  * feedback sensor as the primary encoder.
      //  */
      // armConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     // Set PID values for position control
      //     .p(0.1)
      //     .outputRange(-1, 1)
      //     .maxMotion
      //     // Set MAXMotion parameters for position control
      //     .maxVelocity(2000)
      //     .maxAcceleration(10000)
      //     .allowedClosedLoopError(0.25);

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      
      //  elevatorConfig
      //     .limitSwitch
      //     .reverseLimitSwitchEnabled(true)
      //     .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .d(0.05)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

      // Configure basic settings of the intake motor
      intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      StickConfig.encoder.inverted(false);
      intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.01)
        .d(0)
        .outputRange(-1, 1)
        .maxMotion.maxVelocity(1000)
        .maxAcceleration(60)
        .allowedClosedLoopError(0.5);

      StickConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      StickConfig.encoder.inverted(false);
      StickConfig
        .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.01)
        .d(0)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(200)
        .maxAcceleration(60)
        .allowedClosedLoopError(0.5);
    }
  }

  // public static final class AlgaeSubsystem {
  //   public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
  //   public static final SparkFlexConfig armConfig = new SparkFlexConfig();

  //   static {
  //     // Configure basic setting of the arm motor
  //     armConfig.smartCurrentLimit(40);

  //     /*
  //      * Configure the closed loop controller. We want to make sure we set the
  //      * feedback sensor as the primary encoder.
  //      */
  //     armConfig
  //         .closedLoop
  //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //         // Set PID values for position control. We don't need to pass a closed
  //         // loop slot, as it will default to slot 0.
  //         .p(0.1)
  //         .outputRange(-0.5, 0.5);

  //     // Configure basic settings of the intake motor
  //     intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
  //   }
  // }
}
