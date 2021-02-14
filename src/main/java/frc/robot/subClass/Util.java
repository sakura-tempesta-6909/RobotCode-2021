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

    public enum ColorCode {
        yellow,
        red,
        green,
        blue,
        inRange,
        outOfRange
    }
}
