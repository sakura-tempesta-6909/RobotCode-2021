package frc.robot.autonomous;

import frc.robot.subClass.Const;
import frc.robot.subClass.State;
import frc.robot.subClass.Util;

public class GalacticSearch {

    private GalacticSearchState galacticSearchState;
    private double beforeGyroAngle=0.0 ,beforeSetLeftPosition=0.0,beforeSetRightPosition= 0.0;
    private int positionAchievementCount = 0,angleAchievementCount = 0;
    private double accumulator = 0;
    private boolean isAchievement = false;

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
        state.isResetPID = true;
        accumulator = 0;
        isAchievement = false;
    }

    public void applyState(State state) {
        Util.sendConsole("GalacticSearchStatus1",galacticSearchState.toString());
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
                    intake(state);
                    break;
                case phase3:
                    PIDTurn(90,state);
                    break;
                case phase4:
                    PIDStraight(76.2,state);
                    break;
                case phase5:
                    PIDTurn(-90,state);
                    break;
                case phase6:
                    PIDStraight(152,state);
                    break;
                case phase7:
                    intake(state);
                    break;
                case phase8:
                    PIDTurn(-90, state);
                    break;
                case phase9:
                    PIDStraight(228,state);
                    break;
                case phase10:
                    PIDTurn(90,state);
                    break;
                case phase11:
                    PIDStraight(76.2,state);
                    break;
                case phase12:
                    intake(state);
                    break;
                case phase13:
                    PIDStraight(457,state);
                    break;
                case phase14:
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
                    PIDTurn(-90,state);
                    break;
                case phase5:
                    PIDStraight(228,state);
                    break;
                case phase6:
                    intake(state);
                    break;
                case phase7:
                    PIDTurn(90,state);
                    break;
                case phase8:
                    PIDStraight(152,state);
                    break;
                case phase9:
                    PIDTurn(90,state);
                    break;
                case phase10:
                    PIDStraight(76.2,state);
                    break;
                case phase11:
                    PIDTurn(-90,state);
                    break;
                case phase12:
                    PIDStraight(265,state);
                    break;
                case phase13:
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
        phase14(14),
        finish(-5);
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
            if (positionAchievementCount > 30) {
                phaseInit(state);
                galacticSearchState = galacticSearchState.next();
            }
        }
    }

    private double transformAngle(double angle) {
        angle %= 360;
        if(angle > 180) {
            angle = -(angle-180)+180;
        }
        return angle;
    }

    private void PIDTurn(int targetAngle, State state) {
        state.isTurn = true;
        accumulator += Math.max(-10000, Math.min(transformAngle(targetAngle-transformAngle(state.gyroAngle)), 10000));
        if(!isAchievement) {
            state.driveLeftSetPosition = beforeSetLeftPosition + targetAngle * 4000.0 / 90.0 + accumulator * 0.4;
            state.driveRightSetPosition = beforeSetRightPosition - targetAngle * 4000.0 / 90.0 - accumulator * 0.4;
            angleAchievementCount = 0;
        } else{
            state.driveRightSetPosition = state.driveRightSetPosition + accumulator*0.015;//angleDif(state,targetAngle) * 25;
            state.driveLeftSetPosition = state.driveLeftSetPosition - accumulator * 0.015;
        }
        if(isAngleAchievement(state, targetAngle)){
            if (angleAchievementCount > 10) {
                phaseInit(state);
                galacticSearchState = galacticSearchState.next();
            }
            if(!isAchievement){
                state.driveRightSetPosition = state.driveRightActualPosition + Math.signum(targetAngle) * 20;
                state.driveLeftSetPosition = state.driveLeftActualPosition - Math.signum(targetAngle) * 20;
                accumulator = 0;
            }
            isAchievement = true;
        }
    }

    private void intake(State state) {
        state.intakeState = State.IntakeState.kAutoIntake;
        state.intakeBeltState = State.IntakeBeltState.kIntake;
        state.shooterState = State.ShooterState.kIntake;
        if(state.is_intake_finish) {
            phaseInit(state);
            galacticSearchState = galacticSearchState.next();
        }
    }

    private boolean isPositionAchievement(State state) {
        boolean positionAchievement;
        if(state.isTurn){
            positionAchievement = Util.isPositionAchievement(state.driveRightActualPosition, state.driveRightSetPosition, state.driveLeftActualPosition, state.driveLeftSetPosition,200);
        }else{
            positionAchievement = Util.isPositionAchievement(state.driveRightActualPosition, state.driveRightSetPosition, state.driveLeftActualPosition, state.driveLeftSetPosition);
        }
        if(positionAchievement){
            positionAchievementCount++;
            return true;
        }
        positionAchievementCount = 0;
        return false;
    }

    private boolean isAngleAchievement(State state,int angle){
        Util.sendConsole("angleDif", Math.abs(Math.subtractExact((int) transformAngle(state.gyroAngle - beforeGyroAngle),angle)));
        Util.sendConsole("angle",state.gyroAngle);
        if(Math.abs(Math.subtractExact((int) transformAngle(state.gyroAngle - beforeGyroAngle),angle)) <= 5){
            angleAchievementCount++;
            return true;
        }
        angleAchievementCount = 0;
        return false;
    }

    private double angleDif(State state,int angle){
        return transformAngle(transformAngle(state.gyroAngle - beforeGyroAngle) - angle);
    }
}


