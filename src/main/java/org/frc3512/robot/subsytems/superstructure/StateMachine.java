package org.frc3512.robot.subsytems.superstructure;

import javax.crypto.AEADBadTagException;

import org.frc3512.robot.constants.States;
import org.frc3512.robot.constants.Constants.ArmConstants;
import org.frc3512.robot.constants.Constants.ElevatorConstants;
import org.frc3512.robot.constants.Constants.WristConstants;
import org.frc3512.robot.subsytems.superstructure.arm.Arm;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO.ArmIOInputs;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;
import org.frc3512.robot.util.Position;
import org.frc3512.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase{

    private Position wantedPosition;

    private final ArmIO armIO;
    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

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
            wristIO

        );
    }

    @Override
    public void periodic() {

        synchronized (armInputs) {
            synchronized (elevatorInputs) {
                synchronized (wristInputs) {
                    Logger.processInputs("Subsystems/Superstructure/Arm", armInputs);
                    Logger.processInputs("Subsystems/Superstructure/Elevator", elevatorInputs);
                    Logger.processInputs("Subsystems/Superstructure/Wrist", wristInputs);

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

                    applyStates(wantedPosition);

                }
            }
        }
    }

    public void applyStates(Position position) {

        armIO.setDesiredState(position.getWantedArmState());
        elevatorIO.setDesiredState(position.getWantedElevatorState());
        wristIO.setDesiredState(position.getWantedWristState());

    }

    public boolean reachedSetpoint() {

        synchronized(armInputs) {
            synchronized(elevatorInputs) {
                synchronized(wristInputs) {

                    return MathUtil.isNear(
                            wantedPosition.getWantedArmState(), 
                            armInputs.armAngle,
                            ArmConstants.TOLERANCE)
                        && MathUtil.isNear(
                            wantedPosition.getWantedElevatorState(),
                            elevatorInputs.elevatorHeight, 
                            ElevatorConstants.TOLERANCE)
                        && MathUtil.isNear(
                            wantedPosition.getWantedWristState(),
                            wristInputs.wristAngle, 
                            WristConstants.TOLERANCE);

                }
            }
        }
    }
}