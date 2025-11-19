package org.frc3512.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

  Boolean isEmergencyStopped = DriverStation.isEStopped();

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
  }

  public void setBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * TunerConstants.maxSpeed,
            () -> -controller.getLeftX() * TunerConstants.maxSpeed,
            () -> -controller.getRightX() * TunerConstants.maxAngularRate));

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

    // Mode Switching
    controller.start().onTrue(setCoralMode());
    controller.back().onTrue(setAlgaeMode());

    // Home Robot
    controller.leftBumper().onTrue(homeRobot());

    // Intake
    controller
        .leftTrigger()
        .onTrue(intakeCoral())
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * TunerConstants.slowSpeed,
                () -> -controller.getLeftX() * TunerConstants.slowSpeed,
                () -> -controller.getRightX() * TunerConstants.slowAngularRate));

    // Double Binding Logic
    controller.b().onTrue(runBLogic());
    controller.a().onTrue(runALogic());
    controller.x().onTrue(runXLogic());
    controller.y().onTrue(runYLogic());

    // Execute Action
    controller.rightTrigger().onTrue(maybePlaceCoral()).onFalse(executeAction());
  }

  // * Home
  public Command homeRobot() {
    return Commands.sequence(
        // Reset all mechanisms to stow positions
        arm.changeSetpoint(ArmStates.STOW),
        elevator.changeSetpoint(ElevatorStates.STOW),
        wrist.changeSetpoint(WristStates.STOW),
        intake.changeSetpoint(IntakeStates.STOPPED),
        // Reset scoring variables
        Commands.runOnce(() -> coralReady = false),
        Commands.runOnce(() -> bargeReady = false),
        Commands.runOnce(() -> processorReady = false),
        // Reset states
        Commands.runOnce(() -> wantedLevel = scoringLevels.NONE),
        Commands.runOnce(() -> currentRobotState = States.HOMED),
        // Kill any overlapping commands
        Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()),
        // Log that the commands was run
        logMessage("Homing Robot"),
        // Buzz controller to indicate completion
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1)),
        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
  }

  // * ------ BEGIN CORAL MODE ------

  public Command setCoralMode() {
    return Commands.sequence(
        // Set mode
        Commands.runOnce(() -> currentMode = driverMode.CORAL),
        // Log mode switch
        logMessage("Switched to Coral Mode"),
        // Buzz controller to indicate completion
        Commands.runOnce(() -> controller.setRumble(RumbleType.kRightRumble, 0.75)),
        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> controller.setRumble(RumbleType.kRightRumble, 0)));
  }

  public Command maybeIntakeCoral() {
    return Commands.either(
        // Run full intake Sequence if no coral
        intakeCoral(),
        // Only prep if has coral
        prepCoral(),
        // Logic for if we intake or not
        () -> !haveCoral());
  }

  // intake command (automatic)
  public Command intakeCoral() {
    return Commands.sequence(
        // auto intake coral
        logMessage("Intaking Coral"),
        Commands.runOnce(() -> currentRobotState = States.INTAKING_CORAL),
        arm.changeSetpoint(ArmStates.INTAKE_CORAL),
        elevator.changeSetpoint(ElevatorStates.INTAKE),
        wrist.changeSetpoint(WristStates.INTAKE),
        intake.changeSetpoint(IntakeStates.INTAKE),
        Commands.waitUntil(() -> haveCoral()),
        // prep once we have a coral
        prepCoral());
  }

  // prep coral
  public Command prepCoral() {
    return Commands.sequence(
        logMessage("Prepping Coral"),
        Commands.runOnce(() -> currentRobotState = States.PREPPING_CORAL),
        intake.changeSetpoint(IntakeStates.STOPPED),
        arm.changeSetpoint(ArmStates.HOLD_CORAL),
        elevator.changeSetpoint(ElevatorStates.PREP_CORAL),
        wrist.changeSetpoint(WristStates.CORAL));
  }

  public Command prepTrough() {
    return Commands.sequence(
            logMessage("Prepping Trough"),
            Commands.runOnce(() -> currentRobotState = States.PREPPING_TROUGH),
            Commands.runOnce(() -> wantedLevel = scoringLevels.L1),
            // update to trough prep positions
            arm.changeSetpoint(ArmStates.TROUGH),
            elevator.changeSetpoint(ElevatorStates.TROUGH),
            wrist.changeSetpoint(WristStates.TROUGH),
            // wait until at setpoints
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            // update readiness
            Commands.runOnce(() -> coralReady = true))
        .onlyIf(() -> haveCoral());
  }

  public Command prepL2() {
    return Commands.sequence(
            logMessage("Prepping L2"),
            Commands.runOnce(() -> currentRobotState = States.PREPPING_L2),
            Commands.runOnce(() -> wantedLevel = scoringLevels.L2),
            // update to L2 prep positions
            arm.changeSetpoint(ArmStates.PREP_MID),
            elevator.changeSetpoint(ElevatorStates.PREP_L2),
            wrist.changeSetpoint(WristStates.CORAL),
            // wait until at setpoints
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            // update readiness
            Commands.runOnce(() -> coralReady = true))
        .onlyIf(() -> haveCoral());
  }

  public Command prepL3() {
    return Commands.sequence(
            logMessage("Prepping L3"),
            Commands.runOnce(() -> currentRobotState = States.PREPPING_L3),
            Commands.runOnce(() -> wantedLevel = scoringLevels.L3),
            // update to L3 prep positions
            arm.changeSetpoint(ArmStates.PREP_MID),
            elevator.changeSetpoint(ElevatorStates.PREP_L3),
            wrist.changeSetpoint(WristStates.CORAL),
            // wait until at setpoints
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            // update readiness
            Commands.runOnce(() -> coralReady = true))
        .onlyIf(() -> haveCoral());
  }

  public Command prepL4() {
    return Commands.sequence(
            logMessage("Prepping L4"),
            Commands.runOnce(() -> currentRobotState = States.PREPPING_L4),
            Commands.runOnce(() -> wantedLevel = scoringLevels.L4),
            // update to L4 prep positions
            arm.changeSetpoint(ArmStates.PREP_L4),
            elevator.changeSetpoint(ElevatorStates.PREP_L4),
            wrist.changeSetpoint(WristStates.CORAL),
            // wait until at setpoints
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            // update readiness
            Commands.runOnce(() -> coralReady = true))
        .onlyIf(() -> haveCoral());
  }

  public Command placeTrough() {
    return Commands.either(
        // Place coral if ready
        Commands.sequence(
            logMessage("Placing Trough"),
            Commands.runOnce(() -> currentRobotState = States.PLACING_TROUGH),
            intake.changeSetpoint(IntakeStates.SPIT),
            Commands.waitSeconds(0.5),
            homeRobot()),
        // Prep coral if not ready
        prepCoral(),
        // Boolean check
        () -> coralReady);
  }

  // * spearing logic applies to all levels above l1

  public Command placeL2() {
    return Commands.either(
        // Spear coral if ready
        Commands.sequence(
            logMessage("Placing L2"),
            Commands.runOnce(() -> currentRobotState = States.PLACING_L2),
            arm.changeSetpoint(ArmStates.PLACE_MID),
            elevator.changeSetpoint(ElevatorStates.PLACE_L2)),
        // Prep coral if not ready
        prepCoral(),
        // Boolean check
        () -> coralReady);
  }

  public Command placeL3() {
    return Commands.either(
        Commands.sequence(
            logMessage("Placing L3"),
            Commands.runOnce(() -> currentRobotState = States.PLACING_L3),
            arm.changeSetpoint(ArmStates.PLACE_MID),
            elevator.changeSetpoint(ElevatorStates.PLACE_L3)),
        prepCoral(),
        () -> coralReady);
  }

  public Command placeL4() {
    return Commands.either(
        Commands.sequence(
            logMessage("Placing L4"),
            Commands.runOnce(() -> currentRobotState = States.PLACING_L4),
            arm.changeSetpoint(ArmStates.PLACE_L4),
            elevator.changeSetpoint(ElevatorStates.PLACE_L4)),
        prepCoral(),
        () -> coralReady);
  }

  public Command releaseCoral() {
    // No need for logic as we cannot get here without having already prepped a coral
    return Commands.sequence(
        logMessage("Releasing Coral"),
        intake.changeSetpoint(IntakeStates.PLACE),
        Commands.waitSeconds(0.5),
        homeRobot());
  }

  // we have a maybe since l1 requires no spear on the node,
  // so it will remain prepped in this instance
  public Command maybePlaceCoral() {
    return Commands.either(placeCoral(), Commands.none(), () -> currentMode == driverMode.CORAL);
  }

  // execute the action to score coral based on wanted level
  public Command executeCoralAction() {
    return scoreLogic().onlyIf(() -> coralReady);
  }

  public Command placeCoral() {
    return Commands.either(placeLogic(), prepCoral(), () -> coralReady);
  }

  public Command placeLogic() {
    switch (wantedLevel) {
      case L1:
        return Commands.none();

      case L2:
        return placeL2();

      case L3:
        return placeL3();

      case L4:
        return placeL4();

      default:
        return prepCoral();
    }
  }

  public Command scoreLogic() {
    return Commands.either(placeTrough(), releaseCoral(), () -> wantedLevel == scoringLevels.L1);
  }

  // * ------ BEGIN ALGAE MODE ------

  public Command setAlgaeMode() {
    return Commands.sequence(
        // Set mode
        Commands.runOnce(() -> currentMode = driverMode.ALGAE),
        // Log mode switch
        logMessage("Switched to Algae Mode"),
        // Buzz controller to indicate completion
        Commands.runOnce(() -> controller.setRumble(RumbleType.kLeftRumble, 0.75)),
        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> controller.setRumble(RumbleType.kLeftRumble, 0)));
  }

  public Command maybeIntakeAlgae() {
    return Commands.either(
        // run full intake sequence if no algae
        intakeAlgae(),
        // Only prep if have algae
        prepAlgae(),
        // boolean check
        () -> !haveAlgae());
  }

  public Command intakeAlgae() {
    return Commands.sequence(
        logMessage("Intaking Algae"),
        Commands.runOnce(() -> currentRobotState = States.INTAKING_ALGAE),
        arm.changeSetpoint(ArmStates.INTAKE_ALGAE),
        elevator.changeSetpoint(ElevatorStates.ALGAE_INTAKE),
        wrist.changeSetpoint(WristStates.INTAKE),
        intake.changeSetpoint(IntakeStates.INTAKE),
        Commands.waitUntil(() -> haveAlgae()),
        prepAlgae());
  }

  public Command prepAlgae() {
    return Commands.sequence(
        logMessage("Prepping Algae"),
        Commands.runOnce(() -> currentRobotState = States.HOLDING_ALGAE),
        intake.changeSetpoint(IntakeStates.HOLD),
        arm.changeSetpoint(ArmStates.PREP_ALGAE),
        elevator.changeSetpoint(ElevatorStates.PREP_ALGAE),
        wrist.changeSetpoint(WristStates.ALGAE));
  }

  public Command prepProcess() {
    return Commands.sequence(
            logMessage("Prepping Processor"),
            Commands.runOnce(() -> currentRobotState = States.PREPPING_PROCESSOR),
            // Update to process positions
            arm.changeSetpoint(ArmStates.PROCESS),
            elevator.changeSetpoint(ElevatorStates.PROCESSOR),
            wrist.changeSetpoint(WristStates.PROCESS),
            // Wait until at setpoints
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            // update readiness
            Commands.runOnce(() -> processorReady = true))
        .onlyIf(() -> haveAlgae());
  }

  public Command prepBarge() {
    return Commands.sequence(
            logMessage("Prepping Barge"),
            Commands.runOnce(() -> currentRobotState = States.PREPPING_BARGE),
            // Update to barge positions
            arm.changeSetpoint(ArmStates.BARGE),
            elevator.changeSetpoint(ElevatorStates.BARGE),
            wrist.changeSetpoint(WristStates.ALGAE),
            // Wait until at setpoints
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            // update readiness
            Commands.runOnce(() -> bargeReady = true))
        .onlyIf(() -> haveAlgae());
  }

  public Command process() {
    return Commands.sequence(
            logMessage("Processing Algae"),
            Commands.runOnce(() -> currentRobotState = States.EJECTING),
            intake.changeSetpoint(IntakeStates.EJECT),
            Commands.waitSeconds(0.5),
            homeRobot())
        .onlyIf(() -> haveAlgae() && processorReady);
  }

  public Command placeBarge() {
    return Commands.sequence(
            logMessage("Placing Algae in Barge"),
            Commands.runOnce(() -> currentRobotState = States.EJECTING),
            intake.changeSetpoint(IntakeStates.EJECT),
            Commands.waitSeconds(0.5),
            homeRobot())
        .onlyIf(() -> haveAlgae() && bargeReady);
  }

  public Command grabAlgaeReef(ElevatorStates level) {
    return Commands.either(
        Commands.sequence(
            logMessage("Grabbing Algae from Reef"),
            checkAlgaeLevel(level),
            // begin intake process
            intake.changeSetpoint(IntakeStates.INTAKE),
            arm.changeSetpoint(ArmStates.REMOVE_ALGAE),
            elevator.changeSetpoint(level),
            wrist.changeSetpoint(WristStates.ALGAE),
            // wait until we have algae
            Commands.waitUntil(() -> haveAlgae()),
            // prep algae once we have it
            prepAlgae()),
        prepAlgae(),
        () -> haveAlgae());
  }

  public Command executeAlgaeAction() {
    return Commands.either(process(), placeBarge(), () -> processorReady);
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
        prepL2(), grabAlgaeReef(ElevatorStates.ALGAE_L1), () -> currentMode == driverMode.CORAL);
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

  // * ------ EXCESS LOGIC ------

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  public Command checkAlgaeLevel(ElevatorStates level) {
    if (level == ElevatorStates.ALGAE_L1) {
      return Commands.runOnce(() -> currentRobotState = States.DE_REEFING_A1);
    } else if (level == ElevatorStates.ALGAE_L2) {
      return Commands.runOnce(() -> currentRobotState = States.DE_REEFING_A2);
    } else {
      return Commands.none();
    }
  }

  Boolean haveCoral() {
    if (intake.maybeHasCoral()) {
      return true;
    } else if (intake.hasCoral()) {
      return true;
    } else {
      return false;
    }
  }

  boolean haveAlgae() {
    if (intake.hasAlgae()) {
      return true;
    } else if (intake.isStalled()) {
      return true;
    } else {
      return false;
    }
  }
}
