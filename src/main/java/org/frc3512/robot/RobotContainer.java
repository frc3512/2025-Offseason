package org.frc3512.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.constants.Constants.GeneralConstants;
import org.frc3512.robot.constants.TunerConstants;
import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.arm.ArmIO;
import org.frc3512.robot.subsystems.arm.ArmIOSim;
import org.frc3512.robot.subsystems.arm.ArmIOTalonFX;
import org.frc3512.robot.subsystems.arm.ArmStates;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.drive.GyroIO;
import org.frc3512.robot.subsystems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsystems.drive.ModuleIO;
import org.frc3512.robot.subsystems.drive.ModuleIOSim;
import org.frc3512.robot.subsystems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorIO;
import org.frc3512.robot.subsystems.elevator.ElevatorIOSim;
import org.frc3512.robot.subsystems.elevator.ElevatorIOTalonFX;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeIO;
import org.frc3512.robot.subsystems.intake.IntakeIOSim;
import org.frc3512.robot.subsystems.intake.IntakeIOTalonFX;
import org.frc3512.robot.subsystems.intake.IntakeStates;
import org.frc3512.robot.subsystems.wrist.Wrist;
import org.frc3512.robot.subsystems.wrist.WristIO;
import org.frc3512.robot.subsystems.wrist.WristIOSim;
import org.frc3512.robot.subsystems.wrist.WristIOTalonFX;
import org.frc3512.robot.subsystems.wrist.WristStates;
import org.frc3512.robot.superstructure.States;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  Arm arm;
  Elevator elevator;
  Wrist wrist;

  Intake intake;

  Drive drive;

  // used for score logic, so we have different scoring but one button
  enum scoringLevels {
    L1,

    L2,
    L3,

    L4,

    NONE
  }

  scoringLevels wantedLevel = scoringLevels.NONE;

  Boolean coralReady = false;
  Boolean bargeReady = false;
  Boolean processorReady = false;

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // * Create Mode Selector for single driver
  private enum driverMode {
    CORAL,
    ALGAE
  }

  private driverMode currentMode = driverMode.CORAL;

  private States currentRobotState = States.HOMED;

  // ? Create Vision

  public RobotContainer() {

    switch (GeneralConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        arm = new Arm(new ArmIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());

        intake = new Intake(new IntakeIOTalonFX());

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());

        intake = new Intake(new IntakeIOSim());

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        break;

      default:
        // Replayed robot, disable IO implementations
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});

        intake = new Intake(new IntakeIO() {});

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    setBindings();
  }

  // | Autonomous Logic
  // todo: Implement autonomous command (if worth time)
  public Command getAutonomousCommand() {
    return null;
  }

  // Logic for logging
  public void logData() {
    Logger.recordOutput("Robot States/Current State", currentRobotState);

    Logger.recordOutput("Robot States/Driver Mode", currentMode);

    Logger.recordOutput("Robot States/Scoring Level", wantedLevel);

    Logger.recordOutput("Robot States/Coral Ready", coralReady);
    Logger.recordOutput("Robot States/Barge Ready", bargeReady);
    Logger.recordOutput("Robot States/Processor Ready", processorReady);

    Logger.recordOutput("Mechanism States/Arm State", arm.getState());
    Logger.recordOutput("Mechanism States/Elevator State", elevator.getState());
    Logger.recordOutput("Mechanism States/Wrist State", wrist.getState());
    Logger.recordOutput("Mechanism States/Intake State", intake.getState());
  }

  public void setBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * TunerConstants.maxSpeed,
            () -> -controller.getLeftX() * TunerConstants.maxSpeed,
            () -> -controller.getRightX() * TunerConstants.maxAngularRate));

    // Mode Switching
    controller.start().onTrue(setCoralMode());
    controller.back().onTrue(setAlgaeMode());

    // Reset Gyro
    controller
        .rightStick()
        .and(controller.leftStick())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // * Home Robot
    controller.povDown().onTrue(homeRobot());

    // Intake
    controller.leftTrigger().onTrue(runIntakeLogic());

    // Prep
    controller.b().onTrue(runBLogic());
    controller.a().onTrue(runALogic());
    controller.x().onTrue(runXLogic());
    controller.y().onTrue(runYLogic());

    // Execute Action
    controller.rightTrigger().onTrue(maybePlaceCoral()).onFalse(executeAction());
  }

  // * Home
  public Command homeRobot() {
    return Commands.parallel(
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.STOW)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.STOW)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.STOW)),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.STOPPED)),
        Commands.runOnce(() -> coralReady = false),
        Commands.runOnce(() -> bargeReady = false),
        Commands.runOnce(() -> processorReady = false),
        Commands.runOnce(() -> wantedLevel = scoringLevels.NONE),
        Commands.runOnce(() -> currentRobotState = States.HOMED),
        Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
  }

  // * ------ BEGIN CORAL MODE ------

  public Command setCoralMode() {
    return Commands.runOnce(() -> currentMode = driverMode.CORAL);
  }

  public Command intakeCoral() {
    if (!intake.hasCoral() && !intake.hasAlgae() && !intake.maybeHasCoral()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.INTAKING_CORAL),
          grabCoral(),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.INTAKE_CORAL)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.INTAKE)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.INTAKE)));

    } else {
      return homeRobot();
    }
  }

  public Command grabCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.INTAKE)),
        Commands.waitUntil(() -> intake.hasCoral() || intake.maybeHasCoral()),
        prepCoral());
  }

  public Command prepCoral() {
    return Commands.parallel(
        Commands.runOnce(() -> currentRobotState = States.PREPPING_CORAL),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.STOPPED)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.HOLD_CORAL)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_CORAL)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)));
  }

  public Command prepTrough() {
    if (intake.maybeHasCoral() || intake.hasCoral()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PREPPING_TROUGH),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.TROUGH)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.TROUGH)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.TROUGH)),
          Commands.runOnce(() -> wantedLevel = scoringLevels.L1),
          Commands.runOnce(() -> coralReady = true));
    } else {
      return prepCoral();
    }
  }

  public Command prepL2() {
    if (intake.maybeHasCoral() || intake.hasCoral()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PREPPING_L2),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PREP_MID)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_L2)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
          Commands.runOnce(() -> wantedLevel = scoringLevels.L2),
          Commands.runOnce(() -> coralReady = true));
    } else {
      return prepCoral();
    }
  }

  public Command prepL3() {
    if (intake.maybeHasCoral() || intake.hasCoral()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PREPPING_L3),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PREP_MID)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_L3)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
          Commands.runOnce(() -> wantedLevel = scoringLevels.L3),
          Commands.runOnce(() -> coralReady = true));
    } else {
      return prepCoral();
    }
  }

  public Command prepL4() {
    if (intake.maybeHasCoral() || intake.hasCoral()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PREPPING_L4),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PREP_L4)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_l4)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
          Commands.runOnce(() -> wantedLevel = scoringLevels.L4),
          Commands.runOnce(() -> coralReady = true));
    } else {
      return prepCoral();
    }
  }

  public Command placeTrough() {
    if (coralReady) {
      return Commands.sequence(
          Commands.runOnce(() -> currentRobotState = States.PLACING_TROUGH),
          Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.SPIT)),
          Commands.waitSeconds(0.5),
          homeRobot());
    } else {
      return prepCoral();
    }
  }

  public Command placeL2() {
    if (coralReady) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PLACING_L2),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_MID)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PLACE_L2)));
    } else {
      return prepCoral();
    }
  }

  public Command placeL3() {
    if (coralReady) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PLACING_L3),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_MID)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PLACE_L3)));
    } else {
      return prepCoral();
    }
  }

  public Command placeL4() {
    if (coralReady) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PLACING_L4),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_L4)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PLACE_L4)));
    } else {
      return prepCoral();
    }
  }

  public Command releaseCoral() {
    if (coralReady) {
      return Commands.sequence(
          Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.PLACE)),
          Commands.waitSeconds(0.5),
          homeRobot());
    } else {
      return prepCoral();
    }
  }

  // we have a maybe since l1 requires no spear on the node,
  // so it will remain prepped in this instance
  public Command maybePlaceCoral() {
    if (currentMode == driverMode.CORAL) {
      if (coralReady) {
        switch (wantedLevel) {
          case L1:
            return doNothing();

          case L2:
            return placeL2();

          case L3:
            return placeL3();

          case L4:
            return placeL4();

          case NONE:
            return prepCoral();

          default:
            return doNothing();
        }
      } else {
        return prepCoral();
      }
    } else {
      return homeRobot();
    }
  }

  // execute the action to score coral based on wanted level
  public Command executeCoralAction() {
    if (coralReady) {
      switch (wantedLevel) {
        case L1:
          return placeTrough();

        case L2:
          return releaseCoral();

        case L3:
          return releaseCoral();

        case L4:
          return releaseCoral();

        default:
          return homeRobot();
      }
    } else {
      return prepCoral();
    }
  }

  // * ------ BEGIN ALGAE MODE ------

  public Command setAlgaeMode() {
    return Commands.runOnce(() -> currentMode = driverMode.ALGAE);
  }

  public Command intakeAlgae() {
    if (!intake.hasCoral() && !intake.hasAlgae() && !intake.maybeHasCoral()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.INTAKING_ALGAE),
          grabAlgae(),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.INTAKE_ALGAE)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.INTAKE)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.INTAKE)));
    } else {
      return homeRobot();
    }
  }

  public Command grabAlgae() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.INTAKE)),
        Commands.waitUntil(() -> intake.hasAlgae()),
        prepAlgae());
  }

  public Command prepAlgae() {
    return Commands.parallel(
        Commands.runOnce(() -> currentRobotState = States.HOLDING_ALGAE),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.HOLD)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PREP_ALGAE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_ALGAE)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)));
  }

  public Command prepProcess() {
    if (intake.hasAlgae()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PREPPING_PROCESSOR),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PROCESS)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PROCESSOR)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.PROCESS)),
          Commands.runOnce(() -> processorReady = true));
    } else {
      return prepAlgae();
    }
  }

  public Command prepBarge() {
    if (intake.hasAlgae()) {
      return Commands.parallel(
          Commands.runOnce(() -> currentRobotState = States.PREPPING_BARGE),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.BARGE)),
          Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.BARGE)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)),
          Commands.runOnce(() -> bargeReady = true));
    } else {
      return prepAlgae();
    }
  }

  public Command process() {
    if (processorReady) {
      return Commands.sequence(
          Commands.runOnce(() -> currentRobotState = States.EJECTING),
          Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.EJECT)),
          Commands.waitSeconds(0.5),
          homeRobot());
    } else {
      return prepAlgae();
    }
  }

  public Command placeBarge() {
    if (bargeReady) {
      return Commands.sequence(
          Commands.runOnce(() -> currentRobotState = States.EJECTING),
          Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.EJECT)),
          Commands.waitSeconds(0.5),
          homeRobot());
    } else {
      return prepAlgae();
    }
  }

  public Command grabAlgaeReef(ElevatorStates level) {
    if (!intake.hasAlgae()) {
      return Commands.parallel(
          checkAlgaeLevel(level),
          grabAlgae(),
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.REMOVE_ALGAE)),
          Commands.runOnce(() -> elevator.changeSetpoint(level)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)));
    } else {
      return prepAlgae();
    }
  }

  public Command executeAlgaeAction() {
    if (currentMode == driverMode.ALGAE) {
      if (processorReady) {
        return process();
      } else if (bargeReady) {
        return placeBarge();
      } else {
        return prepAlgae();
      }
    } else {
      return homeRobot();
    }
  }

  // * ------ BEGIN DOUBLE BINDING LOGIC ------

  public Command runIntakeLogic() {
    return Commands.either(intakeCoral(), intakeAlgae(), () -> currentMode == driverMode.CORAL);
  }

  public Command runBLogic() {
    return Commands.either(prepTrough(), prepProcess(), () -> currentMode == driverMode.CORAL);
  }

  public Command runALogic() {
    return Commands.either(
        prepL2(), grabAlgaeReef(ElevatorStates.ALGAE_L2), () -> currentMode == driverMode.CORAL);
  }

  public Command runXLogic() {
    return Commands.either(prepL3(), prepBarge(), () -> currentMode == driverMode.CORAL);
  }

  public Command runYLogic() {
    return Commands.either(
        prepL4(), grabAlgaeReef(ElevatorStates.ALGAE_L2), () -> currentMode == driverMode.CORAL);
  }

  public Command executeAction() {
    return Commands.either(
        executeCoralAction(), executeAlgaeAction(), () -> currentMode == driverMode.CORAL);
  }

  // * ------ EXESS LOGIC ------

  // Used in instances where a case for logic is required but no action is desired
  public Command doNothing() {
    return Commands.runOnce(
        () -> {
          /* nothing */
        });
  }

  public Command checkAlgaeLevel(ElevatorStates level) {
    if (level == ElevatorStates.ALGAE_L1) {
      return Commands.runOnce(() -> currentRobotState = States.DE_REEFING_A1);
    } else if (level == ElevatorStates.ALGAE_L2) {
      return Commands.runOnce(() -> currentRobotState = States.DE_REEFING_A2);
    } else {
      return doNothing();
    }
  }
}