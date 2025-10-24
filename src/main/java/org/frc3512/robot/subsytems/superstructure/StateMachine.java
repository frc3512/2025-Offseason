package org.frc3512.robot.subsytems.superstructure;
import org.frc3512.robot.subsytems.intake.Intake;
import org.frc3512.robot.subsytems.intake.IntakeIO;
import org.frc3512.robot.subsytems.intake.IntakeStates;
import org.frc3512.robot.subsytems.superstructure.arm.Arm;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.elevator.Elevator;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.Wrist;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;

@SuppressWarnings("unused")
public class StateMachine {

    private States wantedState = States.STOW;
    private States currentState;

    private Arm arm = Arm.getInstance();
    private ArmStates currentArmState;
    private Elevator elevator = Elevator.getInstance();
    private ElevatorStates currentElevatorState;
    private Wrist wrist = Wrist.getInstance();
    private WristStates currentWristState;

    private Intake intake = Intake.getInstance();
    private IntakeStates currentIntakeState;

    public StateMachine(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO, IntakeIO intakeIO) {}

    public void handleStateTransitions() {

        switch (wantedState) {
            case INTAKE_CORAL:
                if(!intake.getIntakeIO().hasAlgae() && !intake.getIntakeIO().hasCoral()){
                    currentState = States.INTAKE_CORAL;
                }
            case PREP_TROUGH:
                if(intake.getIntakeIO().hasCoral() && !intake.getIntakeIO().hasAlgae()) {
                    if( /* isAligned()*/  true ){ // isAligned() doesn't exist (its jaydens problem to fix)
                        currentState = States.PLACE_TROUGH;
                    }else{
                        currentState = States.PREP_TROUGH;
                    }
                }
        }
    }

    public void applyStates() {

        switch (currentState) {
            case INTAKE_CORAL:
                currentArmState = ArmStates.INTAKE_CORAL;
                currentElevatorState = ElevatorStates.INTAKE;
                currentWristState = WristStates.INTAKE;
                currentIntakeState = IntakeStates.STOPPED;
            case PREP_TROUGH:
                currentArmState = ArmStates.TROUGH;
                currentElevatorState = ElevatorStates.TROUGH;
                currentWristState = WristStates.TROUGH;
                currentIntakeState = IntakeStates.STOPPED;
            case PLACE_TROUGH:
                currentIntakeState = IntakeStates.EJECT;
        }
                


        arm.setDesiredState(currentArmState);
        elevator.setDesiredState(currentElevatorState);
        wrist.setDesiredState(currentWristState);
        intake.setDesiredState(currentIntakeState);
        
    }

}