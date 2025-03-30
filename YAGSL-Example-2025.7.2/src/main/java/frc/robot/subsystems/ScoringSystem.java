package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Setpoint;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;


public class ScoringSystem
{


  private Elevator    m_elevator;
  private SwerveSubsystem      m_swerve;
  private SwerveInputStream    m_swerveInputStream;
  private TargetingSystem      m_targetSystem;



  public ScoringSystem( Elevator elevator, SwerveSubsystem swerve,
                       TargetingSystem targeting, SwerveInputStream driveStream)
  {
    m_elevator = elevator;
    m_swerve = swerve;
    m_targetSystem = targeting;
    m_swerveInputStream = driveStream;
  }

  ///  Score Coral Command for PathPlanner that does not move the swerve drive at all keeping the pathplanner auto
  /// intact.
  // public Command scoreCoralAuto()
  // {
    // return m_coralArm.setCoralArmAngle(5).repeatedly();
    /*Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
    m_algaeArm.setAlgaeArmAngle(-40).andThen(m_algaeArm.hold()),
                             m_coralArm.getCoralCommand(m_targetSystem).andThen(m_coralArm.hold(false)))
                   .until(m_elevator.atCoralHeight(m_targetSystem).and(m_coralArm.atCoralAngle(m_targetSystem)))
                   .withTimeout(5)
                   .andThen(m_coralIntake.wristScore().alongWith(m_coralArm.getCoralCommand(m_targetSystem)
                                                                           .andThen(m_coralArm.hold(false))),
                                                                           m_algaeArm.setAlgaeArmAngle(-40).andThen(m_algaeArm.hold())
                                         .until(m_coralIntake.atScoringAngle()))
                   .andThen(Commands.parallel(m_coralIntake.wristScore(),
                                              m_elevator.getCoralCommand(m_targetSystem).repeatedly())
                                    .withDeadline(m_coralArm.score()).withTimeout(1)
                                    .until(() -> m_coralArm.coralScored())).andThen(m_coralArm.setCoralArmAngle(-40));*/
  // }

  // public Command restArmsSafe()
  // {
  //   return m_coralArm.setCoralArmAngle(-40).repeatedly().alongWith(m_algaeArm.setAlgaeArmAngle(-40).repeatedly(), m_coralIntake.wristRest());
  // }

  public Command driveCoral()
  {
    // elevator down, drive backwards x in
    return m_targetSystem.driveToCoralTarget(m_swerve);
  }

  public Command scoreCoral()
  {
    // Arm down, elevator down, drive backwards x in
    return Commands.parallel(m_elevator.setSetpointCommand(Setpoint.kFeederStation).repeatedly())
                   .until(m_elevator.L0Trigger())
                   .andThen(m_elevator.setSetpointCommand(Setpoint.kLevel2).repeatedly().withDeadline(m_targetSystem.driveToCoralTarget(m_swerve)))
                   .andThen(m_elevator.SHOOTCommand().repeatedly().until(m_elevator.NOTHoldingCoralTrigger()))
                   .andThen(m_elevator.setSetpointCommand(Setpoint.kFeederStation));
  }

  ///  Autonomous command for scoring the algae arm
  // public Command scoreAlgaeProcessorAuto()
  // {
  //   //set elevator height, set algae angle, spit out ball, drive pose
  //   return Commands.parallel(m_elevator.AlgaePROCESSOR().repeatedly(), m_algaeArm.PROCESSOR().repeatedly()).until(
  //                      m_elevator.aroundAlgaePROCESSOR().and(m_algaeArm.aroundPROCESSORAngle())).withTimeout(5)
  //                  .andThen(Commands.parallel(m_algaeIntake.out(),
  //                                             m_elevator.AlgaePROCESSOR().repeatedly(),
  //                                             m_algaeArm.PROCESSOR().andThen(m_algaeArm.hold()))
  //                                   .until(() -> m_algaeArm.algaeScored())
  //                                   .withTimeout(1));
  // }

//   public Command scoreAlgaeProcessor()
//   {

//     //set elevator height, set algae angle, spit out ball, drive pose
//     return m_swerve.driveToProcessor()
//                    .andThen(Commands.parallel(m_elevator.AlgaePROCESSOR().repeatedly(),
//                                               m_algaeArm.PROCESSOR().repeatedly(),
//                                               m_swerve.lockPos())
//                                     .until(m_elevator.aroundAlgaePROCESSOR()
//                                                      .and(m_algaeArm.aroundPROCESSORAngle()))
//                                     .withTimeout(5))
//                    .andThen(Commands.parallel(m_algaeIntake.out(),
//                                               m_elevator.AlgaePROCESSOR()
//                                                         .repeatedly(),
//                                               m_algaeArm.PROCESSOR().andThen(m_algaeArm.hold()),
//                                               m_swerve.lockPos())
//                                     .until(() -> m_algaeArm.algaeScored()).withTimeout(1));

//   }

//   public Command scoreAlgaeNet()
//   {
//     //set elevator height, set alage angle, spit out ball, drive pose
//     return Commands.parallel(m_algaeArm.NET().andThen(m_algaeArm.hold()), m_elevator.AlgaeNET().repeatedly())
//                    .withTimeout(5).until(m_elevator.aroundAlgaeNET().and(m_algaeArm.aroundNETAngle()))
//                    .andThen(Commands.parallel(m_algaeArm.NET().andThen(m_algaeArm.hold()),
//                                               m_elevator.AlgaeNET().repeatedly(),
//                                               m_algaeIntake.out())
//                                     .withTimeout(2)
//                                     .until(() -> m_algaeArm.algaeScored()));
//   }

}

//                     WALDO
///                     ( ) /-----\
///                  |||||||||
//                   ((o)-(o))
//                    |  W  |
//                    --| |--