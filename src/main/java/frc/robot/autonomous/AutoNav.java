package frc.robot.autonomous;

import frc.robot.subClass.Const;
import frc.robot.subClass.State;
import frc.robot.subClass.Util;

public class AutoNav {

    public AutoNavStatus autoNavStatus;

    public AutoNav(){
        autoNavStatus = AutoNavStatus.waiting;
    }

    public void applyState(State state){
        if(state.autoDriveState == State.AutoDriveState.kAutoNavBlue){

        }else if(state.autoDriveState == State.AutoDriveState.kAutoNavRed){
            switch (autoNavStatus){
                case waiting:
                    autoNavStatus = AutoNavStatus.phase1;
                    break;
                case phase1:
                    state.driveLeftSetPosition = 100 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = 100 * Const.quadraturePositionPerWheelCenti;
                    if(isPositionAchievement(state)){ autoNavStatus = AutoNavStatus.finish;}
                    break;
                case phase2:
                    break;
                case phase3:
                    break;
                case phase4:
                    break;
                case phase5:
                    break;
            }
        }
        Util.sendConsole("AutoNavStatus",autoNavStatus.toString());
    }




    private enum AutoNavStatus{
        waiting,
        phase1,
        phase2,
        phase3,
        phase4,
        phase5,
        finish
    }

    private boolean isPositionAchievement(State state){
       return Util.isPositionAchievement(state.driveRightPosition, state.driveRightSetPosition, state.driveLeftPosition, state.driveLeftSetPosition);
    }
}
