package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.AutoNav;
import frc.robot.autonomous.GalacticSearch;
import frc.robot.subClass.*;

//TalonSRX&VictorSPXのライブラリー

public class Robot extends TimedRobot {

    //コントローラー
    //とりあえず、XboxController2つ
    XboxController driver, operator;

    ADXRS450_Gyro gyro;
    //DriveMotor
    WPI_TalonSRX driveRightFrontMotor, driveLeftFrontMotor;
    VictorSPX driveRightBackMotor, driveLeftBackMotor;

    //ShooterMotor
    TalonSRX shooterLeftMotor, shooterRightMotor;

    //ArmMotor
    TalonSRX armMotor;

    //IntakeMotor
    VictorSPX intakeBeltFrontMotor, intakeBeltBackMotor;
    VictorSPX intakeMotor;

    //climbMotor
    TalonSRX climbMotor;
    Servo climbServo;
    TalonSRX slideMotor;

    //センサー
    DigitalInput intakeFrontSensor, intakeBackSensor;
    SensorCollection armEncoder;
    Servo colorSensorServo;
    Timer servoTimer;
    boolean servoTimerStarted;

    //カメラ
    CameraServer driveCamera, armCamera;

    //SubClass
    Drive drive;
    State state;
    Shooter shooter;
    Intake intake;
    IntakeBelt intakeBelt;
    Arm arm;
    ArmSensor armSensor;
    AutoDrive autoDrive;
    AutoNav autoNav;
    GalacticSearch galacticSearch;

    //モード
    PanelRotationMode panelRotationMode;
    ClimbMode climbMode;

    //Autonomous
    Timer autonomousTimer;
    String gameData;

    @Override
    public void robotInit() {

        //ジャイロセンサー
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        gyro.reset();

        //Intakeセンサー
        intakeFrontSensor = new DigitalInput(Const.IntakeBeltSensorFrontPort);
        intakeBackSensor = new DigitalInput(Const.IntakeBeltSensorBackPort);

        //シューターのモーター
        shooterLeftMotor = new TalonSRX(Const.shooterLeftMotor);
        shooterRightMotor = new TalonSRX(Const.shooterRightMotor);

        //アームのモーター
        armMotor = new TalonSRX(Const.armMotor);
        //アームのセンサー
        armMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        armMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        armEncoder = new SensorCollection(armMotor);

        //カラーセンサーのサーボ
        colorSensorServo = new Servo(Const.colorSensorServoPort);
        servoTimer = new Timer();

        //IntakeBelt
        intakeBeltFrontMotor = new VictorSPX(Const.intakeBeltFrontMotor);
        intakeBeltBackMotor = new VictorSPX(Const.intakeBeltBackMotor);
        intakeMotor = new VictorSPX(Const.IntakeMotorPort);

        //Climb Motor
        climbMotor = new TalonSRX(Const.climbMotorPort);
        climbServo = new Servo(Const.climbServoPort);
        slideMotor = new TalonSRX(Const.climbSlideMotor);
        //魔法の煙を出さないように
        slideMotor.configContinuousCurrentLimit(12);
        slideMotor.enableCurrentLimit(true);

        //コントローラーの初期化
        operator = new XboxController(Const.OperateControllerPort);
        driver = new XboxController(Const.DriveControllerPort);

        //cameraの初期化
        driveCamera = CameraServer.getInstance();
        armCamera = CameraServer.getInstance();
        //driveCamera.startAutomaticCapture("drive", 1);
        //armCamera.startAutomaticCapture("arm", 0);

        //ドライブモーターの初期化
        driveRightFrontMotor = new WPI_TalonSRX(Const.DriveRightFrontPort);
        driveRightBackMotor = new VictorSPX(Const.DriveRightBackPort);
        driveLeftFrontMotor = new WPI_TalonSRX(Const.DriveLeftFrontPort);
        driveLeftBackMotor = new VictorSPX(Const.DriveLeftBackPort);

        //シューターの設定を初期化
        shooterRightMotor.configFactoryDefault();
        shooterLeftMotor.configFactoryDefault();
        //シューターのPIDの設定
        shooterLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Const.kPIDLoopIdx, Const.kTimeoutMs);
        shooterRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Const.kPIDLoopIdx, Const.kTimeoutMs);
        shooterLeftMotor.config_kF(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kF, Const.kTimeoutMs);
        shooterLeftMotor.config_kP(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kP, Const.kTimeoutMs);
        shooterLeftMotor.config_kI(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kI, Const.kTimeoutMs);
        shooterLeftMotor.config_kD(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kD, Const.kTimeoutMs);
        shooterRightMotor.config_kF(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kF, Const.kTimeoutMs);
        shooterRightMotor.config_kP(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kP, Const.kTimeoutMs);
        shooterRightMotor.config_kI(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kI, Const.kTimeoutMs);
        shooterRightMotor.config_kD(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kD, Const.kTimeoutMs);
        shooterLeftMotor.configMaxIntegralAccumulator(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.MaxIntegralAccumulator);
        shooterRightMotor.configMaxIntegralAccumulator(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.MaxIntegralAccumulator);
        shooterRightMotor.setSensorPhase(true);
        shooterLeftMotor.setSensorPhase(true);

        //Armの設定を初期化
        armMotor.configFactoryDefault();
        //ArmのPID設定
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Const.kArmPIDLoopIdx, Const.kTimeoutMs);
        armMotor.config_kF(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kF, Const.kTimeoutMs);
        armMotor.config_kP(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kP, Const.kTimeoutMs);
        armMotor.config_kI(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kI, Const.kTimeoutMs);
        armMotor.config_kD(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kD, Const.kTimeoutMs);
        armMotor.configMaxIntegralAccumulator(Const.kPIDLoopIdx, Const.kGains_ArmPosition.MaxIntegralAccumulator);
        // エンコーダの反転修正
        armMotor.setSensorPhase(true);
        // モーターの反転修正
        armMotor.setInverted(true);

        // 初期化
        driveLeftFrontMotor.configFactoryDefault();
        driveRightFrontMotor.configFactoryDefault();
        driveRightBackMotor.configFactoryDefault();
        driveLeftBackMotor.configFactoryDefault();
        // エンコーダ
        driveLeftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Const.kArmPIDLoopIdx, Const.kTimeoutMs);
        driveRightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Const.kArmPIDLoopIdx, Const.kTimeoutMs);
        // エンコーダの反転修正
        driveLeftFrontMotor.setSensorPhase(true);
        driveRightFrontMotor.setSensorPhase(true);
        // フォローの設定
        driveRightBackMotor.follow(driveRightFrontMotor);
        driveLeftBackMotor.follow(driveLeftFrontMotor);

        //サブクラスの生成
        armSensor = new ArmSensor(armMotor);
        arm = new Arm(armMotor, armEncoder, armSensor);
        shooter = new Shooter(shooterRightMotor, shooterLeftMotor);
        intake = new Intake(intakeMotor);
        intakeBelt = new IntakeBelt(intakeBeltFrontMotor, intakeBeltBackMotor, intakeFrontSensor, intakeBackSensor);
    
        autoNav = new AutoNav();
        galacticSearch = new GalacticSearch();

        state = new State();

        //モードのクラスの生成
        panelRotationMode = new PanelRotationMode(colorSensorServo);
        climbMode = new ClimbMode(climbMotor, climbServo, slideMotor);
    }

    @Override
    public void robotPeriodic() {
        //処理なし
    }

    @Override
    public void autonomousInit() {
        // 初期化
        driveLeftFrontMotor.configFactoryDefault();
        driveRightFrontMotor.configFactoryDefault();
        driveRightBackMotor.configFactoryDefault();
        driveLeftBackMotor.configFactoryDefault();


        // エンコーダ
        driveLeftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Const.kArmPIDLoopIdx, Const.kTimeoutMs);
        driveRightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Const.kArmPIDLoopIdx, Const.kTimeoutMs);
        // エンコーダの反転修正
        driveLeftFrontMotor.setSensorPhase(true);
        driveRightFrontMotor.setSensorPhase(true);
        // PIDだけ右が逆に動くので反転
        driveRightFrontMotor.setInverted(true);
        driveRightBackMotor.setInverted(true);

        // PID値設定
        driveLeftFrontMotor.config_kF(Const.kArmPIDLoopIdx, 0, Const.kTimeoutMs);
        driveLeftFrontMotor.config_kP(Const.kArmPIDLoopIdx, 0.2, Const.kTimeoutMs);
        driveLeftFrontMotor.config_kI(Const.kArmPIDLoopIdx, 0, Const.kTimeoutMs);
        driveLeftFrontMotor.config_kD(Const.kArmPIDLoopIdx, 0, Const.kTimeoutMs);


        driveRightFrontMotor.config_kF(Const.kArmPIDLoopIdx, 0, Const.kTimeoutMs);
        driveRightFrontMotor.config_kP(Const.kArmPIDLoopIdx, 0.2, Const.kTimeoutMs);
        driveRightFrontMotor.config_kI(Const.kArmPIDLoopIdx, 0, Const.kTimeoutMs);
        driveRightFrontMotor.config_kD(Const.kArmPIDLoopIdx, 0, Const.kTimeoutMs);

        // フォローの設定
        driveRightBackMotor.follow(driveRightFrontMotor);
        driveLeftBackMotor.follow(driveLeftFrontMotor);

        // エンコーダの値リセット
        driveRightFrontMotor.getSensorCollection().setQuadraturePosition(0, 10);
        driveLeftFrontMotor.getSensorCollection().setQuadraturePosition(0, 10);

        autoDrive = new AutoDrive(driveLeftFrontMotor,driveRightFrontMotor);

        // タイマーリセット&スタート
        autonomousTimer = new Timer();
        autonomousTimer.reset();
        autonomousTimer.start();

        // DriverStationから情報受取り
        gameData = DriverStation.getInstance().getGameSpecificMessage();

        panelRotationMode.contractServo();

        // state初期化
        state.stateInit();
        state.controlMode = State.ControlMode.m_Auto;
        state.autoDriveState = State.AutoDriveState.kAutoNavDoNothing;
        autoNav.autoNavStatus = AutoNav.AutoNavStatus.waiting;
        arm.applyState(state);
        shooter.applyState(state);
        intake.applyState(state);
        intakeBelt.applyState(state);
        galacticSearch.applyState(state);
        autoNav.applyState(state);
        autoDrive.applyState(state);
    }

    @Override
    public void autonomousPeriodic() {
        Util.isPositionAchievement(state.driveRightPosition, state.driveRightSetPosition, state.driveLeftPosition, state.driveLeftSetPosition);
        state.driveRightPosition = autoDrive.getRightMotorPosition();
        state.driveLeftPosition = autoDrive.getLeftMotorPosition();
        Util.sendConsole("LeftPosition",state.driveLeftPosition+"");
        Util.sendConsole("RightPosition",state.driveRightPosition+"");
        //Util.sendConsole("SetPosition",state.driveLeftSetPosition+"");
        //Util.sendConsole("AutoDriveMode",state.autoDriveState.toString());
        state.autoDriveState = State.AutoDriveState.kAutoNavRed;
<<<<<<< HEAD
        state.gyroAngle = gyro.getAngle();
        state.gyroRate = gyro.getRate();
        // drive.applyState(state);
=======
>>>>>>> 369fc80930b8ac8f48750de5df9fbaf56e432222
        arm.applyState(state);
        shooter.applyState(state);
        intake.applyState(state);
        intakeBelt.applyState(state);
        galacticSearch.applyState(state);
        autoNav.applyState(state);
        autoDrive.applyState(state);
        System.out.println("LeftSetPosition" + state.driveLeftSetPosition);
        System.out.println("RightSetPosition" + state.driveRightSetPosition);
        System.out.println("  ");
    }


    public void teleopInit() {
        state.controlMode = State.ControlMode.m_Drive;
        panelRotationMode.contractServo();
        
        // 初期化
        driveRightFrontMotor.configFactoryDefault();
        driveLeftFrontMotor.configFactoryDefault();
        driveRightBackMotor.configFactoryDefault();
        driveLeftBackMotor.configFactoryDefault();
        
        // フィードバックセンサーの追加
        driveLeftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        driveRightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        //ドライブモーターの台形加速
        driveLeftFrontMotor.configOpenloopRamp(Const.DriveFullSpeedTime);
        driveRightFrontMotor.configOpenloopRamp(Const.DriveFullSpeedTime);

        // フォロー
        driveLeftBackMotor.follow(driveLeftFrontMotor);
        driveRightBackMotor.follow(driveRightFrontMotor);

        drive = new Drive(driveLeftFrontMotor, driveRightFrontMotor);
    }

    @Override
    public void disabledInit() {
        //super.disabledInit();
         state.stateInit();
        // drive.applyState(state);
        // arm.applyState(state);
         shooter.applyState(state);
         intake.applyState(state);
         intakeBelt.applyState(state);
    }

    @Override
    public void teleopPeriodic() {

        //System.out.println("leftSpeed" + shooterLeftMotor.getSelectedSensorVelocity());
        //System.out.println("RightSpeed" + shooterRightMotor.getSelectedSensorVelocity());

        //状態初期化
        state.stateInit();
        state.armAngle = arm.getArmNow();
        state.shooterLeftMotorSpeed = shooter.getMotorSpeed(true);
        state.shooterRightMotorSpeed = shooter.getMotorSpeed(false);
        
        //Mode Change
        switch (state.controlMode) {
            case m_Drive:
                if (operator.getBumper(GenericHID.Hand.kLeft)) {
                    //O LB ボール発射モードへ切り替え
                    state.controlMode = State.ControlMode.m_ShootingBall;
                } else if (driver.getBackButton()) {
                    //D Back コントロールパネル回転モードへ切り替え
                    state.controlMode = State.ControlMode.m_PanelRotation;
                    state.armTargetAngle = state.armAngle;
                    //state.armTargetAngle = State.armAngle;
                    //state.targetAngle = State.armAngle;
                } else if (operator.getBackButton()) {
                    //O Backクライムモードへ切り替え
                    state.controlMode = State.ControlMode.m_Climb;
                }
                break;

            case m_Climb:
            case m_ShootingBall:
                if (driver.getStartButton() || operator.getStartButton()) {
                    //D or O Start ドライブモードへ
                    state.controlMode = State.ControlMode.m_Drive;
                    state.climbWireState = State.ClimbWireState.climbLock;
                }
                break;
            case m_PanelRotation:
                if (driver.getStartButton() || operator.getStartButton()) {
                    //D or O Start ドライブモードへ
                    panelRotationMode.contractServo();
                    if (!servoTimerStarted) {
                        servoTimer.reset();
                        servoTimer.start();
                        servoTimerStarted = true;
                    }
                    System.out.println(colorSensorServo.getAngle());
                    if (servoTimer.get() > 1) {
                        state.controlMode = State.ControlMode.m_Drive;
                        servoTimerStarted = false;
                    }
                }
                break;
        }
        System.out.println(colorSensorServo.getAngle());
        Util.sendConsole("Mode", state.controlMode.toString());

        switch (state.controlMode) {
            case m_Drive:
                //ほかに関係なくドライブ
                if (driver.getBumper(GenericHID.Hand.kLeft)) {
                    //D LBで低速モード
                    state.driveState = State.DriveState.kLow;
                } else {
                    state.driveState = State.DriveState.kManual;
                }

                state.driveStraightSpeed = Util.deadbandProcessing(-driver.getY(GenericHID.Hand.kLeft));
                state.driveRotateSpeed = Util.deadbandProcessing(driver.getX(GenericHID.Hand.kRight));

                // 常にローラーを回しておくかどうかを制御
                if (driver.getYButton()) {
                    // D Y  ローラー回すかを切り替える
                    state.is_intakeRollInDrive = !state.is_intakeRollInDrive;
                }
                if (state.is_intakeRollInDrive) {
                    state.intakeState = State.IntakeState.kDrive;
                } else {
                    state.intakeState = State.IntakeState.doNothing;
                }

                if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kLeft))) {
                    //D LT ボールを取り込む
                    state.intakeState = State.IntakeState.kIntake;
                    state.intakeBeltState = State.IntakeBeltState.kIntake;
                    state.shooterState = State.ShooterState.kIntake;
                } else if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //D RT ボールを出す
                    state.intakeState = State.IntakeState.kOuttake;
                    state.intakeBeltState = State.IntakeBeltState.kOuttake;
                    state.shooterState = State.ShooterState.kOuttake;
                } else if (driver.getBumper(GenericHID.Hand.kRight)) {
                    //D RB アームを平行（パネルくぐり）
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Const.armParallelAngle;
                    state.intakeState = State.IntakeState.doNothing;
                }
                break;

            case m_ShootingBall:
                //Armは調整しない限り動かない
                state.armState = State.ArmState.k_Conserve;
                //D Stick ドライブを少し動かす
                state.driveState = State.DriveState.kLow;
                state.driveStraightSpeed = -driver.getY(GenericHID.Hand.kLeft);
                state.driveRotateSpeed = driver.getX(GenericHID.Hand.kRight);
                if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //O RT ボールを飛ばす
                    state.shooterState = State.ShooterState.kShoot;
                    state.intakeBeltState = State.IntakeBeltState.kOuttake;
                } else if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))) {
                    //O LT 60度に角度調整//
                    state.armState = State.ArmState.k_ConstAng;
                    state.armFinalTargetAngle = 35;
                } else if (operator.getBButton()) {
                    //O B Red
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Util.pointToAngle(Const.interStellarRedPoint);
                } else if (operator.getYButton()) {
                    //O Y　Yellow
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Util.pointToAngle(Const.interStellarYellowPoint);
                } else if (operator.getAButton()) {
                    //O A　Green
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Util.pointToAngle(Const.interstellarGreenPoint);
                } else if (operator.getXButton()) {
                    //O X　Blue
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Util.pointToAngle(Const.interStellarBluePoint);
                } else if (operator.getBumper(GenericHID.Hand.kRight)) {
                    //O RB PowerPortChallenge
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Util.pointToAngle(Const.PowerPortChallengePoint);
                }else if (Util.deadbandCheck(operator.getY(GenericHID.Hand.kLeft))) {
                    //O LStick Y 砲台の角度を手動で調節, 正か負のみ
                    state.armState = State.ArmState.k_Adjust;
                    state.armMotorSpeed = -operator.getY(GenericHID.Hand.kLeft);
                }
                    /*if(Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))&&Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))){
                        state.intakeBeltState = State.IntakeBeltState.kouttake;
                    }*/
                break;

            case m_Climb:
                //Drive
                state.driveState = State.DriveState.kLow;
                state.driveStraightSpeed = Util.deadbandProcessing(-driver.getY(GenericHID.Hand.kLeft));
                state.driveRotateSpeed = Util.deadbandProcessing(driver.getX(GenericHID.Hand.kRight));

                //Climb
                state.armState = State.ArmState.k_Conserve;
                if (operator.getYButton()) {
                    //O Y アームを上げる
                    state.climbArmState = State.ClimbArmState.climbExtend;
                    state.climbExtendAdjustSpeed = -operator.getY(GenericHID.Hand.kLeft) / 3.5;
                } else if (operator.getBButton()) {
                    //O B クライムする(本番)
                    state.climbWireState = State.ClimbWireState.climbShrink;
                } else if (Util.deadbandCheck(operator.getX(GenericHID.Hand.kRight))) {
                    //O RStick X スライド
                    state.climbArmState = State.ClimbArmState.climbSlide;
                    if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))) {
                        //O LT 高出力でスライド
                        state.climbSlideMotorSpeed = -operator.getX(GenericHID.Hand.kLeft);
                    } else {
                        //1/2の出力でスライド
                        state.climbSlideMotorSpeed = operator.getX(GenericHID.Hand.kRight) / 2;
                    }
                }
                if (operator.getBumper(GenericHID.Hand.kRight)) {
                    //O RB Climb Motorだけ縮める(調整用)
                    state.climbWireState = State.ClimbWireState.climbMotorOnlyShrink;
                } else if (operator.getBumper(GenericHID.Hand.kLeft)) {
                    //O LB Climb　Motorだけ伸ばす(調整用)
                    state.climbWireState = State.ClimbWireState.climbMotorOnlyExtend;
                } else if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //O RT ロックする
                    state.climbWireState = State.ClimbWireState.climbLock;
                }

                climbMode.changeState(state);
                break;

            case m_PanelRotation:
                //Drive
                state.driveStraightSpeed = Util.deadbandProcessing(-driver.getY(GenericHID.Hand.kLeft));
                state.driveRotateSpeed = Util.deadbandProcessing(driver.getX(GenericHID.Hand.kRight));

                //Arm
                state.armState = State.ArmState.k_PID;
                state.armSetAngle = Const.armPanelAngle;

                if (driver.getXButton()) {
                    //D X 青に合わせる
                    state.panelState = State.PanelState.p_toBlue;
                } else if (driver.getYButton()) {
                    //D Y 黄に合わせる
                    state.panelState = State.PanelState.p_toYellow;
                } else if (driver.getBButton()) {
                    //D B 赤に合わせる
                    state.panelState = State.PanelState.p_toRed;
                } else if (driver.getAButton()) {
                    //D A 緑に合わせる
                    state.panelState = State.PanelState.p_toGreen;
                } else if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kLeft))) {
                    //D LT 手動左回転
                    state.panelState = State.PanelState.p_ManualRot;
                    state.panelManualSpeed = -Const.shooterPanelManualSpeed;
                } else if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //D RT 手動右回転
                    state.panelState = State.PanelState.p_ManualRot;
                    state.panelManualSpeed = Const.shooterPanelManualSpeed;
                }
                panelRotationMode.changeState(state);
                break;
        }

        drive.applyState(state);
        arm.applyState(state);
        shooter.applyState(state);
        intake.applyState(state);
        intakeBelt.applyState(state);

    }

    @Override
    public void testInit() {
        driveRightFrontMotor.getSensorCollection().setQuadraturePosition(0, 10);
        driveLeftFrontMotor.getSensorCollection().setQuadraturePosition(0, 10);

    }

    @Override
    public void testPeriodic() {
        Util.sendConsole("LeftPosition", driveLeftFrontMotor.getSelectedSensorPosition()+"");
        Util.sendConsole("RightPosition",driveRightFrontMotor.getSelectedSensorPosition()+"");
    }
}