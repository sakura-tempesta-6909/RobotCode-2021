package frc.robot.autonomous;

import frc.robot.subClass.Const;
import frc.robot.subClass.State;
import frc.robot.subClass.Util;

public class AutoNav {

    public AutoNavState autoNavStatus;
    private double beforeGyroAngle=0.0 ,beforeSetLeftPosition=0.0,beforeSetRightPosition= 0.0;
    private int positionAchievementCount = 0,angleAchievementCount = 0;

    public AutoNav() {
        autoNavStatus = AutoNavState.waiting;
    }

    public void applyState(State state) {
        if (state.autoDriveState == State.AutoDriveState.kAutoNavBlue) {
        } else if (state.autoDriveState == State.AutoDriveState.kAutoNavRed) {
            switch (autoNavStatus) {
                case waiting:
                    phaseInit(state);
                    autoNavStatus = AutoNavState.phase1;
                    break;
                case phase1:

                    break;
                case phase2:

                    break;
                case phase3:
                    break;
                case phase4:
                    break;
                case phase5:
                    break;
                case finish:
                    break;
            }
        }
        //Util.sendConsole("AutoNavStatus", autoNavStatus.toString());
    }


    public enum AutoNavState {
        waiting,
        phase1,
        phase2,
        phase3,
        phase4,
        phase5,
        finish
    }

    private void phaseInit(State state){
        state.intakeState = State.IntakeState.doNothing;
        state.intakeBeltState = State.IntakeBeltState.doNothing;
        state.shooterState = State.ShooterState.doNothing;
        beforeGyroAngle = state.gyroAngle;
        beforeSetRightPosition = state.driveRightSetPosition;
        beforeSetLeftPosition = state.driveLeftSetPosition;
        state.is_intake_finish = false;
        positionAchievementCount = 0;
        state.loopPeakOutput = 0.8;
    }
    private boolean isPositionAchievement(State state) {
        boolean positionAchievement = Util.isPositionAchievement(state.driveRightActualPosition, state.driveRightSetPosition, state.driveLeftActualPosition, state.driveLeftSetPosition);
        if(positionAchievement){
            positionAchievementCount++;
            return true;
        }
        positionAchievementCount = 0;
        return false;
    }

    private boolean isAngleAchievement(State state,int angle){
        if(Math.abs(Math.subtractExact(Math.round(Math.abs(state.gyroAngle - beforeGyroAngle)),angle)) < 5){
            angleAchievementCount++;
            return true;
        }
        angleAchievementCount = 0;
        return false;
    }
}