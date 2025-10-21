package org.frc3512.robot.subsytems.superstructure;

import javax.crypto.AEADBadTagException;

import org.frc3512.robot.constants.States;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;
import org.frc3512.robot.util.Position;
import org.frc3512.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase{

    private Position wantedPosition;

    private final ArmIO armIO;
    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private final ArmIOInputsAutoLogged armInputs = new ArmInputsIOAutoLogged();
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public enum WantedState {
        STOW,
        IDLE,
        MOVE_TO_POS,
    }

    private enum SystemState {
        STOWING,
        IDLING,
        MOVING_TO_POS,
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public StateMachine(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO) {
        this.armIO = armIO;
        this.armIO = armIO;
        this.wristIO = wristIO;

        wantedPosition = States.STOWED;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
                () -> {
                    synchronized (armInputs) {
                        synchronized (elevatorInputs) {
                            synchronized (wristInputs) {
                                armIO.updateInputs(armInputs);
                                elevatorIO.updateInputs(elevatorInputs);
                                wristIO.updateInputs(wristInputs);
                            }
                        }
                    }
                },
                armIO,
                elevatorIO,
                wristIO);
    }

    @Override
    public void periodic() {
        synchronized (armInputs) {
            synchronized (elevatorInputs) {
                synchronized (wristInputs) {
                    Logger.processInputs("Subsystems/Superstructure/Arm", armInputs);
                    Logger.processInputs("Subsystems/Superstructure/Elevator", elevatorInputs);
                    Logger.processInputs("Subsystems/Superstructure/Wrist", wristInputs);

                    systemState = handleStateTransitions();

                    Logger.recordOutput("Subsystems/Superstructure/SystemState", systemState);
                    Logger.recordOutput("Subsystems/Superstructure/WantedState", wantedState);
                    Logger.recordOutput("Subsystems/Superstructure/ReachedSetpoint", atSetpoint());

                    ArmStates wantedArmPos;
                    ElevatorStates wantedElevatorPos;
                    WristStates wantedWristPos;

                    if (wantedPosition != null) {
                        wantedArmPos = wantedPosition.getWantedArmState();
                        wantedElevatorPos = wantedPosition.getWantedElevatorState();
                        wantedWristPos = wantedPosition.getWantedWristState();
                        Logger.recordOutput("Subsystems/Superstructure/Wanted Arm Pos", wantedArmPos);
                        Logger.recordOutput("Subsystems/Superstructure/Wanted Elevator Pos", wantedElevatorPos);
                        Logger.recordOutput("Subsystems/Superstructure/Wanted Wrist Pos", wantedWristPos);
                    }

                    applyStates();

                    previousWantedState = this.wantedState;
                }
            }
        }
    }
}
