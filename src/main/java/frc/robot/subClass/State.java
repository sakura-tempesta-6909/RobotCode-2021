package frc.robot.subClass;

public class State {

    //速度
    //Drive
    /** driveStraightSpeed, driveRotateSpeed それぞれDriveMode時の真っ直ぐ進む成分, 回転する成分(PercentOutput) [-1, 1] */
    public double driveStraightSpeed, driveRotateSpeed;
    public double driveRightSetPosition,driveLeftSetPosition;
    public double driveRightActualPosition, driveLeftActualPosition;
    public double loopPeakOutput;
    public boolean isTurn;

    public double gyroAngle;
    public double gyroRate;

    //Shooter
    /** panelManualSpeed PanelRotationMode時の回転スピード.普通定数の入力 (単位不明)[範囲不明だが-1,1の予感] */
    public double panelManualSpeed;

    /** shooterLeftSpeed, shooterRightSpeed それぞれPanelRotationMode時の左右のスピードの成分. 基本的にpanelManualSpeedの数字を入れる (単位不明)[範囲不明だが-1,1の予感] */
    public double shooterLeftSpeed, shooterRightSpeed;

    /**  shooterLeftMotorSpeed, shooterRightMotorSpeed それぞれShootingBallMode時の左右のスピードの成分. (PercentOutput)[-1, 1] */
    public double shooterLeftMotorSpeed, shooterRightMotorSpeed;


    //Arm
    /** armMotorSpeed アームを回す速さ(PercentOutput)[-1, 1] */
    public double armMotorSpeed;


    //Climb
    /**  climbSlideMotorSpeed Climbした後に左右にスライドするときのスピード. (PercentOutput)[-1, 1] */
    public double climbSlideMotorSpeed;

    /** climbExtendAdjustSpeed ClimbMode時のアームを上げるスピード(PercentOutput)[-1, 1] */
    public double climbExtendAdjustSpeed;


    //Arm Angle
    /** armAgnle アームの実際の角度 (度数法)[-30, 80] */
    public double armAngle;

    /** armSetAngle PID制御時の目標角度 (度数法)[-30, 80] */
    public double armSetAngle;

    /** armFinalTargetAngle 立崎追加分 PID制御のアームの最終的な目標値 (度数法)[-30, 80] */
    public double armFinalTargetAngle;

    /** armTargetAngle 立崎追加分 PID制御の仮の目標値 (度数法)[-30, 80] */
    public double armTargetAngle;
    

    //SubClass State
    public DriveState driveState;
    public ArmState armState;
    public ShooterState shooterState;
    public IntakeState intakeState;
    public IntakeBeltState intakeBeltState;
    public ClimbArmState climbArmState;
    public ClimbWireState climbWireState;
    public PanelState panelState;
    public AutoDriveState autoDriveState;

    //Control Mode
    public ControlMode controlMode = ControlMode.m_Drive;

    /** is_intakeFull ボールを5個ゲットしたか */
    public boolean is_intakeFull;
    public boolean is_intake_finish;
    /** is_intakeRollInDrive m_Drive時、インテイクを回すモードにしているかどうか */
    public boolean is_intakeRollInDrive = true;

    public State() {
        stateInit();
        controlMode = ControlMode.m_Drive;
        is_intakeRollInDrive = true;
    }

    public void stateInit() {

        //Drive
        driveState = DriveState.kManual;
        driveStraightSpeed = 0;
        driveRotateSpeed = 0;

        driveLeftSetPosition = 0;
        driveRightSetPosition = 0;

        driveLeftActualPosition = 0;
        driveRightActualPosition = 0;

        isTurn = false;

        loopPeakOutput = 0.8;

        gyroAngle = 0;
        gyroRate = 0;

        autoDriveState = AutoDriveState.kAutoNavDoNothing;

        //Shooter
        shooterState = ShooterState.doNothing;
        shooterLeftSpeed = 0;
        shooterRightSpeed = 0;
        shooterRightMotorSpeed = 0;
        shooterLeftMotorSpeed = 0;

        //Intake
        intakeState = IntakeState.doNothing;
        is_intakeFull = false;
        is_intake_finish = false;

        //IntakeBeltState
        intakeBeltState = IntakeBeltState.doNothing;

        //Climb
        climbArmState = ClimbArmState.doNothing;
        climbWireState = ClimbWireState.doNothing;
        climbSlideMotorSpeed = 0;
        climbExtendAdjustSpeed = 0;

        //Arm
        armState = ArmState.k_Basic;
        armMotorSpeed = 0;
        armSetAngle = Const.armMinAngle;
        armAngle = 0;
        armTargetAngle = 0;

        //panel
        panelState = PanelState.p_DoNothing;
        panelManualSpeed = 0;
    }

    public enum ControlMode {
        m_ShootingBall,
        m_PanelRotation,
        m_Climb,
        m_Drive,
        m_Auto
    }

    public enum DriveState {
        kManual,
        kLow,
        kSuperLow,
        kStop,
        kMiddleLow
    }

    public enum AutoDriveState{
        kAutoNavBlue,
        kGalacticSearchBlue,
        kAutoNavRed,
        kGalacticSearchRed,
        kAutoNavDoNothing,
        kGalacticSearchDoNothing
    }

    public enum ShooterState {
        kShoot,
        kIntake,
        kManual,
        doNothing,
        kOuttake
    }

    public enum IntakeState {
        kIntake,
        kOuttake,
        doNothing,
        kDrive
    }

    public enum IntakeBeltState {
        kIntake,
        kOuttake,
        doNothing
    }

    public enum ClimbArmState {
        doNothing,
        climbExtend,
        climbSlide
    }

    public enum ClimbWireState {
        doNothing,
        climbMotorOnlyExtend,
        climbMotorOnlyShrink,
        climbShrink,
        climbLock
    }

    public enum ArmState {
        k_Conserve,
        k_Adjust,
        k_PID,
        k_Basic,
        k_Manual,
        k_ConstAng,
        k_DoNothing,
    }

    public enum PanelState {
        p_DoNothing,        //stop
        p_ManualRot,        //手動
        p_toBlue,           //色合わせ(大会のパネル側のセンサーの色なのでカラーセンサーが読み取るのは二つずれた値。青<->赤、黄<->緑）
        p_toYellow,
        p_toRed,
        p_toGreen

    }

}