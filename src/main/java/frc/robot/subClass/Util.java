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
     * @param nowAngle 現在の角度(度数法)[-30, 80]
     * 
     * @return FeedForward 重力オフセット(PercentOutput)[-1, 1]
     */
    public static double getFeedForward(double nowAngle) {
        return Const.armMaxOffset * Math.cos(Math.toRadians(nowAngle));
    }

    /** 
     * エンコーダの値(point)を角度に変換.
     * 
     * @param point エンコーダの値[160付近, 500付近]
     * 
     * @return angle 角度(度数法)[-30, 80]
     * 
     */
    public static double pointToAngle(double point) {
        return  (point - Const.armMinPoint) * Const.armAngleDifference / Const.armPointDifference + Const.armMinAngle;
    }

    /** 
     * 角度をエンコーダの値(point)に変換.
     * 
     * @param angle 角度(度数法)[-30, 80]
     * 
     * @return point エンコーダの値[160付近, 500付近]
     */
    public static double angleToPoint(double angle) {
        return (angle - Const.armMinAngle) * Const.armPointDifference /
                Const.armAngleDifference + Const.armMinPoint;
    }
}
