package frc.robot.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subClass.Const;
import frc.robot.subClass.State;

public class AutoDrive {

    private final WPI_TalonSRX leftMotor,rightMotor;

    public AutoDrive(WPI_TalonSRX leftMotor, WPI_TalonSRX rightMotor){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void applyState(State state) {
        if (state.controlMode == State.ControlMode.m_Auto) {
            switch (state.autoDriveState){
                case kAutoNavRed:
                case kAutoNavBlue:
                case kGalacticSearchRed:
                case kGalacticSearchBlue:
                    setPosition(state.driveLeftSetPosition,state.driveRightSetPosition);
                    break;
                case kGalacticSearchDoNothing:
                case kAutoNavDoNothing:
                    break;
            }
        }
    }

    private void setPosition(double driveLeftSetPosition,double driveRightSetPosition){
        leftMotor.set(ControlMode.Position, driveLeftSetPosition);
        rightMotor.set(ControlMode.Position, driveRightSetPosition);
    }

    public double getLeftMotorPosition(){
        return leftMotor.getSelectedSensorPosition();
    }

    public double getRightMotorPosition(){
        return leftMotor.getSelectedSensorPosition();
    }
}
