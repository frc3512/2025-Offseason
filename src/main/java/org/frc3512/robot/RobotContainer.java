package org.frc3512.robot;

import org.frc3512.robot.buttons.ControlBoard;
import org.frc3512.robot.buttons.ModeControls;
import org.frc3512.robot.constants.DriveConstants;
import org.frc3512.robot.constants.DriveConstants.TunerSwerveDrivetrain;
import org.frc3512.robot.subsytems.drive.Swerve;
import org.frc3512.robot.subsytems.superstructure.StateMachine;
import org.frc3512.robot.subsytems.superstructure.arm.Arm;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIOTalonFX;
import org.frc3512.robot.subsytems.superstructure.elevator.Elevator;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIOTalonFX;
import org.frc3512.robot.subsytems.superstructure.wrist.Wrist;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIOTalonFX;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@SuppressWarnings("unused")
public class RobotContainer {

  private double maxSpeed = DriveConstants.maxSpeed;
  private double maxAngularRate = DriveConstants.maxAngularRate;

  // * Create subsytem objects
  private Arm arm;
  private Elevator elevator;
  private Wrist wrist;

  private Swerve drivetrain = DriveConstants.createDrivetrain();
  
  private StateMachine stateMachine;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.07) // * Add a 7% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // * Create Buttons
  private final ControlBoard buttons = ControlBoard.getInstance();

  // ? Create Vision

  public RobotContainer() {

    // * Initialize Subsystems
    Arm.setInstance(new ArmIOTalonFX());
    arm = Arm.getInstance();

    Elevator.setInstance(new ElevatorIOTalonFX());
    elevator = Elevator.getInstance();

    Wrist.setInstance(new WristIOTalonFX());
    wrist = Wrist.getInstance();

    configureAxisActions();

  }

  private void configureAxisActions() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(buttons.getThrottle() * maxSpeed)
                    .withVelocityY(buttons.getStrafe() * maxSpeed)
                    .withRotationalRate(buttons.getRotation() * maxAngularRate)));
  }
  
}
