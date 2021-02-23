package frc.robot.autonomous;

import frc.robot.subClass.Const;
import frc.robot.subClass.State;
import frc.robot.subClass.Util;

public class AutoNav {

    public AutoNavState autoNavStatus;
    private Double beforeGyroAngle = 0.0;

    public AutoNav() {
        autoNavStatus = AutoNavState.waiting;
    }

    public void applyState(State state) {
        if (state.autoDriveState == State.AutoDriveState.kAutoNavBlue) {

        } else if (state.autoDriveState == State.AutoDriveState.kAutoNavRed) {
            switch (autoNavStatus) {
                case waiting:
                    autoNavStatus = AutoNavState.phase1;
                    break;
                case phase1:
                    state.driveLeftSetPosition = 1000 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = 1000 * Const.quadraturePositionPerWheelCenti;
                    if (isPositionAchievement(state)) {
                        phaseInit(state);
                        autoNavStatus = AutoNavState.phase2;
                    }
                    break;
                case phase2:
                    state.intakeState = State.IntakeState.kIntake;
                    state.intakeBeltState = State.IntakeBeltState.kIntake;
                    state.shooterState = State.ShooterState.kIntake;
                    if(state.is_intake_finish){
                        phaseInit(state);
                        autoNavStatus = AutoNavState.phase3;
                    }
                    break;
                case phase3:
                    if(Math.abs(state.gyroAngle - beforeGyroAngle)>85){
                        phaseInit(state);
                        autoNavStatus = AutoNavState.finish;
                    }
                    state.driveLeftSetPosition = 100 * Const.quadraturePositionPerWheelCenti;
                    state.driveRightSetPosition = -100 * Const.quadraturePositionPerWheelCenti;
                    break;
                case phase4:
                    break;
                case phase5:
                    break;
                case finish:
                    state.intakeState = State.IntakeState.doNothing;
                    state.intakeBeltState = State.IntakeBeltState.doNothing;
                    state.shooterState = State.ShooterState.doNothing;
                    break;
            }
        }
        Util.sendConsole("AutoNavStatus", autoNavStatus.toString());
        Util.sendConsole("GyroStatus", state.gyroAngle - beforeGyroAngle+"");
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
        state.is_intake_finish = false;
    }
    private boolean isPositionAchievement(State state) {
        return Util.isPositionAchievement(state.driveRightActualPosition, state.driveRightSetPosition, state.driveLeftActualPosition, state.driveLeftSetPosition);
    }
}