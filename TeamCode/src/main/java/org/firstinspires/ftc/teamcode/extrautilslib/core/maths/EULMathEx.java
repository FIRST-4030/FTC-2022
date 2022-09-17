package org.firstinspires.ftc.teamcode.extrautilslib.core.maths;

public class EULMathEx {

    public enum Axis{
        AXIS_X,
        AXIS_Y,
        AXIS_Z,
    }

    public static final double PI = Math.PI;
    public static final double TAU = Math.PI * 2;
    public static final double GOLDEN_RATIO = (1 + Math.sqrt(5))/2;

    public static final double DEG2RAD = PI / 180;
    public static final double RAD2DEG = 180 / PI;

    public static double doubleClamp(double min, double max, double value){
        return Math.min(max, Math.max(min, value));
    }
}
