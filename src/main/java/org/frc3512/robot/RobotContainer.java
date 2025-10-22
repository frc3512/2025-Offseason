package org.frc3512.robot;

import org.frc3512.robot.subsytems.superstructure.arm.Arm;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIOTalonFX;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // * Create subsytem objects
  private Arm arm;

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // ? Create Vision

  public RobotContainer() {

    // * Initialize Subsystems
    Arm.setInstance(new ArmIOTalonFX());
    arm = Arm.getInstance();

  }

  private void setCoralBindings() {}

  private void setAlgaeBindings() {}
  
}
