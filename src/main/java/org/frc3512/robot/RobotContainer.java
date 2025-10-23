package org.frc3512.robot;

import org.frc3512.robot.subsytems.superstructure.StateMachine;
import org.frc3512.robot.subsytems.superstructure.arm.Arm;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIOTalonFX;
import org.frc3512.robot.subsytems.superstructure.elevator.Elevator;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIOTalonFX;
import org.frc3512.robot.subsytems.superstructure.wrist.Wrist;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIOTalonFX;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@SuppressWarnings("unused")
public class RobotContainer {

  // * Create subsytem objects
  private Arm arm;
  private Elevator elevator;
  private Wrist wrist;
  
  private StateMachine stateMachine;

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // ? Create Vision

  public RobotContainer() {

    // * Initialize Subsystems
    Arm.setInstance(new ArmIOTalonFX());
    arm = Arm.getInstance();

    Elevator.setInstance(new ElevatorIOTalonFX());
    elevator = Elevator.getInstance();

    Wrist.setInstance(new WristIOTalonFX());
    wrist = Wrist.getInstance();

    StateMachine.setInstance(
      Arm.getInstance().getArmIO(),
      Elevator.getInstance().getElevatorIO(),
      Wrist.getInstance().getWristIO()
    );
    stateMachine = StateMachine.getInstance();

  }
  
}
