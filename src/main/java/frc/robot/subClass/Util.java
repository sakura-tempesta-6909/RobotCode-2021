package frc.robot.subClass;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Util {

    //不感帯処理
    public static double deadbandProcessing(double value) {
        return Math.abs(value) > Const.Deadband ? value : 0;
    }

    public static boolean deadbandCheck(double value) {
        return Math.abs(value) > Const.Deadband;
    }


    public static void sendConsole(String key, String text) {
        System.out.println(key + ":" + text);
        SmartDashboard.putString(key, text);
    }

    /** 
     * 目標角度に合わせた重力オフセットを計算.
     * 
     * <p> (地面と水平な時の重力オフセット) * (cos現在角度)
     * 
     * @param nowAngle 現在の角度
     */
    public static double getFeedForward(double nowAngle) {
        return Const.armMaxOffset * Math.cos(Math.toRadians(nowAngle));
    }

    public static double pointToAngle(double point) {
        return  (point - Const.armMinPoint) * Const.armAngleDifference / Const.armPointDifference + Const.armMinAngle;
    }

    /** 
     * 目標角度に合わせたPIDの目標値を計算.
     * 
     * <p> (角度の目標値最小値差分) *（エンコーダー値の最大最小差分) / (角度の最大最小差分) + (最小値からの差分)
     * 
     * @param angle 角度の目標値
     */
    public static double angleToPoint(double angle) {
        return (angle - Const.armMinAngle) * Const.armPointDifference /
                Const.armAngleDifference + Const.armMinPoint;
    }

    
}
