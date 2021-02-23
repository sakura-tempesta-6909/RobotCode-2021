package frc.robot.autonomous;

import frc.robot.subClass.Const;
import frc.robot.subClass.State;
import frc.robot.subClass.Util;

public class GalacticSearch {

    private GalacticSearchState galacticSearchState;
    private double beforeGyroAngle=0.0 ,beforeSetLeftPosition=0.0,beforeSetRightPosition= 0.0;
    private int positionAchievementCount = 0,angleAchievementCount = 0;

    public GalacticSearch() {
        galacticSearchState = GalacticSearchState.waiting;
    }

    public void applyState(State state) {
        if (state.autoDriveState == State.AutoDriveState.kGalacticSearchRed) {

        } else if (state.autoDriveState == State.AutoDriveState.kGalacticSearchBlue) {
            switch (galacticSearchState) {
                case waiting:
                    phaseInit(state);
                    galacticSearchState = GalacticSearchState.phase1;
                    break;
                case phase1:
                    state.driveLeftSetPosition = beforeSetLeftPosition + 457 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition + 457 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        if (positionAchievementCount >10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase2;
                        }
                    }
                    break;
                case phase2:
                case phase10:
                    state.intakeState = State.IntakeState.kIntake;
                    state.intakeBeltState = State.IntakeBeltState.kIntake;
                    state.shooterState = State.ShooterState.kIntake;
                    if(state.is_intake_finish){
                        phaseInit(state);
                        galacticSearchState = GalacticSearchState.phase3;
                    }
                    break;
                case phase3:
                    state.driveLeftSetPosition = beforeSetLeftPosition + 70 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition + 70 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        if (positionAchievementCount >10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase2;
                        }
                    }
                    break;
                case phase4:
                case phase11:
                    if(isAngleAchievement(state,90)){
                        if (angleAchievementCount == 1) {
                            state.driveRightSetPosition = state.driveRightActualPosition;
                            state.driveLeftSetPosition = state.driveLeftActualPosition;
                            beforeSetLeftPosition = state.driveLeftSetPosition;
                            beforeSetRightPosition = state.driveRightSetPosition;
                        } else if (angleAchievementCount > 10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase4;
                        }
                    }else{
                        state.loopPeakOutput = 0.5;
                        state.driveLeftSetPosition =  beforeSetLeftPosition + (90 - (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
                        state.driveRightSetPosition = beforeSetRightPosition - (90- (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
                        beforeSetRightPosition = state.driveRightSetPosition;
                        beforeSetLeftPosition = state.driveLeftSetPosition;
                    }
                    break;
                case phase5:
                    state.driveLeftSetPosition = beforeSetLeftPosition + 220 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition + 220 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        if (positionAchievementCount >10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase5;
                        }
                    }
                    break;
                case phase6:
                    if(isAngleAchievement(state,-90)){
                    if (angleAchievementCount == 1) {
                        state.driveRightSetPosition = state.driveRightActualPosition;
                        state.driveLeftSetPosition = state.driveLeftActualPosition;
                        beforeSetLeftPosition = state.driveLeftSetPosition;
                        beforeSetRightPosition = state.driveRightSetPosition;
                    } else if (angleAchievementCount > 10) {
                        phaseInit(state);
                        galacticSearchState = GalacticSearchState.phase4;
                    }
                }else{
                    state.loopPeakOutput = 0.5;
                    state.driveLeftSetPosition =  beforeSetLeftPosition + (-90 - (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition - (-90- (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
                    beforeSetRightPosition = state.driveRightSetPosition;
                    beforeSetLeftPosition = state.driveLeftSetPosition;
                }
                    break;
                case phase7:
                    state.driveLeftSetPosition = beforeSetLeftPosition + 150 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition + 150 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        if (positionAchievementCount >10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase5;
                        }
                    }
                    break;
                case phase8:
                    if(isAngleAchievement(state,-90)){
                        if (angleAchievementCount == 1) {
                            state.driveRightSetPosition = state.driveRightActualPosition;
                            state.driveLeftSetPosition = state.driveLeftActualPosition;
                            beforeSetLeftPosition = state.driveLeftSetPosition;
                            beforeSetRightPosition = state.driveRightSetPosition;
                        } else if (angleAchievementCount > 10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase4;
                        }
                    }else{
                        state.loopPeakOutput = 0.5;
                        state.driveLeftSetPosition =  beforeSetLeftPosition + (-90 - (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
                        state.driveRightSetPosition = beforeSetRightPosition - (-90- (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
                        beforeSetRightPosition = state.driveRightSetPosition;
                        beforeSetLeftPosition = state.driveLeftSetPosition;
                    }
                case phase9:
                    state.driveLeftSetPosition = beforeSetLeftPosition + 70 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition + 70 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        if (positionAchievementCount >10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase2;
                        }
                    }
                    break;
                case phase12:
                    state.driveLeftSetPosition = beforeSetLeftPosition + 150 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = beforeSetRightPosition + 150 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        if (positionAchievementCount >10) {
                            phaseInit(state);
                            galacticSearchState = GalacticSearchState.phase2;
                        }
                    }
                    break;
                case phase13:
                    state.intakeState = State.IntakeState.kOuttake;
                    state.intakeBeltState = State.IntakeBeltState.kOuttake;
                    state.shooterState = State.ShooterState.kOuttake;
                    break;
                case finish:
                    state.intakeState = State.IntakeState.doNothing;
                    state.intakeBeltState = State.IntakeBeltState.doNothing;
                    state.shooterState = State.ShooterState.doNothing;
                    break;
            }
        }
    }

    public enum GalacticSearchState {
        waiting,
        phase1,
        phase2,
        phase3,
        phase4,
        phase5,
        phase6,
        phase7,
        phase8,
        phase9,
        phase10,
        phase11,
        phase12,
        phase13,
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


