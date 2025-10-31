package org.frc3512.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.constants.Constants.GeneralConstants;
import org.frc3512.robot.constants.TunerConstants;
import org.frc3512.robot.subsytems.arm.Arm;
import org.frc3512.robot.subsytems.arm.ArmIO;
import org.frc3512.robot.subsytems.arm.ArmIOSim;
import org.frc3512.robot.subsytems.arm.ArmIOTalonFX;
import org.frc3512.robot.subsytems.drive.Drive;
import org.frc3512.robot.subsytems.drive.GyroIO;
import org.frc3512.robot.subsytems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsytems.drive.ModuleIO;
import org.frc3512.robot.subsytems.drive.ModuleIOSim;
import org.frc3512.robot.subsytems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsytems.elevator.Elevator;
import org.frc3512.robot.subsytems.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.elevator.ElevatorIOSim;
import org.frc3512.robot.subsytems.elevator.ElevatorIOTalonFX;
import org.frc3512.robot.subsytems.intake.Intake;
import org.frc3512.robot.subsytems.intake.IntakeIO;
import org.frc3512.robot.subsytems.intake.IntakeIOSim;
import org.frc3512.robot.subsytems.intake.IntakeIOTalonFX;
import org.frc3512.robot.subsytems.wrist.Wrist;
import org.frc3512.robot.subsytems.wrist.WristIO;
import org.frc3512.robot.subsytems.wrist.WristIOSim;
import org.frc3512.robot.subsytems.wrist.WristIOTalonFX;
import org.frc3512.robot.superstructure.Superstructure;

@SuppressWarnings("unused")
public class RobotContainer {

  private Arm arm;
  private Elevator elevator;
  private Wrist wrist;

  private Intake intake;

  private Drive drive;

  private Superstructure superstructure = new Superstructure();

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // ? Create Vision

  // * Create Mode Selector for single driver
  private enum driverMode {
    CORAL,
    ALGAE
  }

  private driverMode currentMode;

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

    configureBindings();
  }

  private void configureAxisActions() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> getThrottle(), () -> getStrafe(), () -> getRotation()));
  }

  private void configureButtonBindings() {

    // Reset gyro to 0° when B button is pressed
    resetGyro()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Mode Switcher
    switchToCoral()
        .onTrue(
            Commands.runOnce(() -> setMode(driverMode.CORAL)));
    switchToAlgae()
        .onTrue(
            Commands.runOnce(() -> setMode(driverMode.ALGAE)));

  }

  private void configureBindings() {
    configureAxisActions();
    configureButtonBindings();
  }

  private void setMode(driverMode mode) {
    switch (mode) {
      case CORAL:
        currentMode = driverMode.CORAL;
        break;
      case ALGAE:
        currentMode = driverMode.ALGAE;
        break;
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }

  //  * Define triggers here

  // Swerve
  // Use exponential joystick for more acceleration control
  // Linear: input = output
  // Exponential: greater input = greater output
  private double getThrottle() {
    return -(Math.pow(Math.abs(controller.getLeftY()), 1.2)) * Math.signum(controller.getLeftY());
  }

  private double getStrafe() {
    return -(Math.pow(Math.abs(controller.getLeftX()), 1.2)) * Math.signum(controller.getLeftX());
  }

  private double getRotation() {
    return -(Math.pow(Math.abs(controller.getRightX()), 1.5)) * Math.signum(controller.getRightX());
  }

  // Gyro
  private Trigger resetGyro() {
    return controller.rightStick().and(controller.leftStick());
  }

  // Superstructure
  private Trigger stow() {
    return controller.povDown();
  }

  // Mode Switcher
  private Trigger switchToCoral() {
    return controller.start();
  }

  private Trigger switchToAlgae() {
    return controller.back();
  }

  //  --- CORAL ---
  private Trigger intakeCoral() {
    if (currentMode == driverMode.CORAL) {
      return controller.leftTrigger();
    } else {
      return null;
    }
  }

  private Trigger l1() {
    if (currentMode == driverMode.CORAL) {
      return controller.b();
    } else {
      return null;
    }
  }

  private Trigger l2() {
    if (currentMode == driverMode.CORAL) {
      return controller.a();
    } else {
      return null;
    }
  }

  private Trigger l3() {
    if (currentMode == driverMode.CORAL) {
      return controller.x();
    } else {
      return null;
    }
  }

  private Trigger l4() {
    if (currentMode == driverMode.CORAL) {
      return controller.y();
    } else {
      return null;
    }
  }

  // Score method in superstructure will need logic to determoine if we score on mid or l4, because they 
  // will have differnt scoring methods due to differnt branch shapes
  private Trigger score() {
    if (currentMode == driverMode.CORAL) {
      return controller.rightTrigger();
    } else {
      return null;
    }
  }

  // --- ALGAE ---
  private Trigger intakeAlgae() {
    if (currentMode == driverMode.ALGAE) {
      return controller.leftTrigger();
    } else {
      return null;
    }
  }

  private Trigger deReefA1() {
    if (currentMode == driverMode.ALGAE) {
      return controller.a();
    } else {
      return null;
    }
  }

  private Trigger deReefA2() {
    if (currentMode == driverMode.ALGAE) {
      return controller.y();
    } else {
      return null;
    }
  }

  private Trigger process() {
    if (currentMode == driverMode.ALGAE) {
      return controller.b();
    } else {
      return null;
    }
  }

  private Trigger barge() {
    if (currentMode == driverMode.ALGAE) {
      return controller.x();
    } else {
      return null;
    }
  }

  // Vision - Implement after IO if fully working
  private Trigger allignLeft() {
    if (currentMode == driverMode.CORAL) {
      return controller.leftBumper();
    } else {
      return null;
    }
  }

  private Trigger allignRight() {
    if (currentMode == driverMode.CORAL) {
      return controller.rightBumper();
    } else {
      return null;
    }
  }

  private Trigger allignAlgae() {
    if (currentMode == driverMode.ALGAE) {
      return controller.leftBumper();
    } else {
      return null;
    }
  }
}
