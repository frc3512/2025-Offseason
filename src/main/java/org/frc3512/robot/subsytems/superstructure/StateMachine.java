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


    public StateMachine(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO) {

        this.armIO = armIO;
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;

        wantedPosition = States.STOWED;

    }

    @Override
    public void periodic() {
        applyStates(wantedPosition);
    }

    public void applyStates(Position position) {

        armIO.setDesiredState(position.getWantedArmState());
        elevatorIO.setDesiredState(position.getWantedElevatorState());
        wristIO.setDesiredState(position.getWantedWristState());

    }

}