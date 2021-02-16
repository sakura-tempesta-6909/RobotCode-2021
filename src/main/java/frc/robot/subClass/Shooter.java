package frc.robot.subClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter {
    TalonSRX shooterRight, shooterLeft;

    public Shooter(TalonSRX shooterRight, TalonSRX shooterLeft) {
        this.shooterLeft = shooterLeft;
        this.shooterRight = shooterRight;
    }

    public void applyState(State state) {
        switch (state.shooterState) {
            case kShoot:
                setSpeed(-1.0, 1.0);
                break;
            case kIntake:
                setSpeedPercent(Const.shooterIntakeSpeed, -Const.shooterIntakeSpeed);
                break;
            case kOuttake:
                setSpeedPercent(Const.shooterOutTakeSpeed, -Const.shooterOutTakeSpeed);
                break;
            case kManual:
                setSpeed(state.shooterLeftSpeed, state.shooterRightSpeed);
                break;
            case doNothing:
                setSpeed(0, 0);
                shooterLeft.setIntegralAccumulator(0);
                shooterRight.setIntegralAccumulator(0);
                break;
        }
    }

    /**
     * シューターを動かす
     * 
     * @param leftSpeed 左のシューターのスピード。正で取り込む方向。(単位不明)[範囲不明だが-1,1の予感]
     * @param rightSpeed 右のシューターのスピード。負で取り込む方向。(単位不明)[範囲不明だが-1,1の予感]
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        double targetLeftVelocity_UnitsPer100ms = leftSpeed * Const.shooterMotorMaxOutput;
        double targetRightVelocity_UnitsPer100ms = rightSpeed * Const.shooterMotorMaxOutput;
        shooterLeft.set(ControlMode.Velocity, targetLeftVelocity_UnitsPer100ms);
        shooterRight.set(ControlMode.Velocity, targetRightVelocity_UnitsPer100ms);
    }

    /**
     * シューターを動かす
     * 
     * @param speedPercentLeft 左のシューターのスピード。正で取り込む方向。 (PercentOutput)[-1, 1]
     * @param speedPercentRight 右のシューターのスピード。負で取り込む方向。 (PercentOutput)[-1, 1]
     */
    public void setSpeedPercent(double speedPercentLeft, double speedPercentRight) {
        shooterLeft.set(ControlMode.PercentOutput, speedPercentLeft);
        shooterRight.set(ControlMode.PercentOutput, speedPercentRight);

    }

    /**
     * 左のシューターのスピード取得
     * 
     * @return 左のシューターのスピード (単位不明)[範囲不明だが10万で最大近い模様]
     */
    public double getMotorSpeed() {
        return getMotorSpeed(true);
    }
    /**
     * シューターのスピード取得
     * 
     * @param isLeft 左なのか右なのか
     * 
     * @return 選ばれたシューターのスピード(単位不明)[範囲不明だが10万で最大近い模様]
     */
    public double getMotorSpeed(boolean isLeft) {
        if (isLeft) {
            return shooterLeft.getSelectedSensorVelocity();
        } else {
            return shooterRight.getSelectedSensorVelocity();
        }
    }

}
