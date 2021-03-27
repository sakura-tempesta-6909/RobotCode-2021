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

    private void phaseInit(State state){
        state.intakeState = State.IntakeState.doNothing;
        state.intakeBeltState = State.IntakeBeltState.doNothing;
        state.shooterState = State.ShooterState.doNothing;
        beforeGyroAngle = state.gyroAngle;
        beforeSetRightPosition = state.driveRightSetPosition;
        beforeSetLeftPosition = state.driveLeftSetPosition;
        state.is_intake_finish = false;
        state.isTurn = false;
        positionAchievementCount = 0;
        state.loopPeakOutput = 0.8;
    }

    public void applyState(State state) {
        System.out.println("statusssss"+galacticSearchState.toString());
        if (state.autoDriveState == State.AutoDriveState.kGalacticSearchRed) {
            switch (galacticSearchState){
                case waiting:
                    phaseInit(state);
                    galacticSearchState = GalacticSearchState.phase1;
                    break;
                case phase1:
                    PIDStraight(152,state);
                    break;
                case phase2:
                case phase6:
                case phase11:
                    intake(state);
                    break;
                case phase3:
                    PIDTurn(-90,state);
                    break;
                case phase4:
                    PIDStraight(76.2,state);
                    break;
                case phase5:
                    PIDStraight(152,state);
                case phase7:
                    PIDTurn(90,state);
                    break;
                case phase8:
                    PIDStraight(228,state);
                    break;
                case phase9:
                    PIDTurn(90,state);
                    break;
                case phase10:
                    PIDStraight(76.2,state);
                    break;
                case phase12:
                    PIDStraight(457,state);
                    break;
                case phase13:
                    state.intakeState = State.IntakeState.doNothing;
                    state.intakeBeltState = State.IntakeBeltState.doNothing;
                    state.shooterState = State.ShooterState.doNothing;
                    galacticSearchState = GalacticSearchState.finish;
                    break;
            }
        } else if (state.autoDriveState == State.AutoDriveState.kGalacticSearchBlue) {
            switch (galacticSearchState) {
                case waiting:
                    phaseInit(state);
                    galacticSearchState = GalacticSearchState.phase1;
                    break;
                case phase1:
                    PIDStraight(457, state);
                    break;
                case phase2:
                    intake(state);
                    break;
                case phase3:
                    PIDStraight(76.2, state);
                    break;
                case phase4:
                    PIDTurn(90,state);
                    break;
                case phase5:
                    PIDStraight(228,state);
                    break;
                case phase6:
                    intake(state);
                    break;
                case phase7:
                    PIDTurn(333,state);
                    break;
                case phase8:
                    PIDStraight(170,state);
                    break;
                case phase9:PIDTurn(-27,state);
                    break;
                case phase10:
                    PIDStraight(152,state);
                    break;
                case phase12:
                    galacticSearchState = GalacticSearchState.finish;
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
        waiting(0),
        phase1(1),
        phase2(2),
        phase3(3),
        phase4(4),
        phase5(5),
        phase6(6),
        phase7(7),
        phase8(8),
        phase9(9),
        phase10(10),
        phase11(11),
        phase12(12),
        phase13(13),
        finish(14);
        private final int id;

        GalacticSearchState(int id) {this.id = id;}

        public GalacticSearchState next() {
            final GalacticSearchState[] all = GalacticSearchState.values();
            for(GalacticSearchState state:all) {
                if(state.id == this.id+1) {
                    return state;
                }
            }
            return phase2;
        }
    }

    private void PIDStraight(double relativePosition, State state) {
        state.driveLeftSetPosition = beforeSetLeftPosition + relativePosition * Const.quadraturePositionPerWheelCenti;
        state.driveRightSetPosition = beforeSetRightPosition + relativePosition * Const.quadraturePositionPerWheelCenti;
        state.isTurn = false;
        if (isPositionAchievement(state)) {
            if (positionAchievementCount > 10) {
                phaseInit(state);
                galacticSearchState = galacticSearchState.next();
            }
        }
    }

    private void PIDTurn(int targetAngle, State state) {
        state.isTurn = true;
        if(isAngleAchievement(state,targetAngle)){
            if (angleAchievementCount == 1) {
                state.driveRightSetPosition = state.driveRightActualPosition;
                state.driveLeftSetPosition = state.driveLeftActualPosition;
                beforeSetLeftPosition = state.driveLeftSetPosition;
                beforeSetRightPosition = state.driveRightSetPosition;
            } else if (angleAchievementCount > 10) {
                phaseInit(state);
                //galacticSearchState = galacticSearchState.next();
            }
        }else{
            state.loopPeakOutput = 0.5;
            state.driveLeftSetPosition =  beforeSetLeftPosition + (targetAngle - (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
            state.driveRightSetPosition = beforeSetRightPosition - (targetAngle - (state.gyroAngle - beforeGyroAngle)) * 0.8 * Const.quadraturePositionPerWheelCenti;
            beforeSetRightPosition = state.driveRightSetPosition;
            beforeSetLeftPosition = state.driveLeftSetPosition;
        }
    }

    private void intake(State state) {
        state.intakeState = State.IntakeState.kIntake;
        state.intakeBeltState = State.IntakeBeltState.kIntake;
        state.shooterState = State.ShooterState.kIntake;
        if(state.is_intake_finish){
            phaseInit(state);
            galacticSearchState = galacticSearchState.next();
        }

    }

    private void outtake(State state) {
        state.intakeState = State.IntakeState.kOuttake;
        state.intakeBeltState = State.IntakeBeltState.kOuttake;
        state.shooterState = State.ShooterState.kOuttake;
    }

    private boolean isPositionAchievement(State state) {
        boolean positionAchievement = Util.isPositionAchievement(state.driveRightActualPosition, state.driveRightSetPosition, state.driveLeftActualPosition, state.driveLeftSetPosition);
        if(positionAchievement){
            positionAchievementCount++;
            Util.sendConsole("yessss","kehffohifwfiwefwijfi2ur8yyru3ru3ru3ru3");
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


