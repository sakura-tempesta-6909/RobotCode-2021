package frc.robot.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subClass.Const;
import frc.robot.subClass.State;
import frc.robot.subClass.Util;

public class AutoDrive {

    private final WPI_TalonSRX leftMotor, rightMotor;

    public AutoDrive(WPI_TalonSRX leftMotor, WPI_TalonSRX rightMotor){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void applyState(State state) {
        if (state.controlMode == State.ControlMode.m_Auto) {
            if (state.isResetPID){
                leftMotor.setIntegralAccumulator(0);
                rightMotor.setIntegralAccumulator(0);
                state.isResetPID = false;
            }
            if(state.isTurn) {
                leftMotor.selectProfileSlot(Const.kDriveTurnPIDLoopIdx, 0);
                rightMotor.selectProfileSlot(Const.kDriveTurnPIDLoopIdx, 0);
            }else{
                leftMotor.selectProfileSlot(Const.kDriveStraightPIDLoopIdx, 1);
                rightMotor.selectProfileSlot(Const.kDriveStraightPIDLoopIdx, 1);
            }
            switch (state.autoDriveState){
                case kGalacticSearchDoNothing:
                case kAutoNavDoNothing:
                    break;
                default:
                    setPosition(state.driveLeftSetPosition,state.driveRightSetPosition);
                    break;
            }
        }
        //System.out.println("left_velocity" + leftMotor.getSelectedSensorVelocity());
        //System.out.println("right_velocity" + rightMotor.getSelectedSensorVelocity());
    }

    private void setPosition(double driveLeftSetPosition,double driveRightSetPosition){
        leftMotor.set(ControlMode.Position, driveLeftSetPosition);
        rightMotor.set(ControlMode.Position, driveRightSetPosition);
    }

    public double getLeftMotorPosition(){
        return leftMotor.getSelectedSensorPosition();
    }

    public double getRightMotorPosition(){
        return rightMotor.getSelectedSensorPosition();
    }
}
