package org.firstinspires.ftc.teamcode.extrautilslib.core.maths;

public class EULMathEx {

    public enum Axis{
        AXIS_X,
        AXIS_Y,
        AXIS_Z,
    }

    public static double doubleClamp(double min, double max, double value){
        return Math.min(max, Math.max(min, value));
    }

    public static float floatClamp(float min, float max, float value){
        return Math.min(max, Math.max(min, value));
    }

    public static <T extends Comparable<T>> T clamp(T min, T max, T value){
        T clampUpper = max.compareTo(value) > 0 ? value : max;
        return min.compareTo(clampUpper) < 0 ? min : clampUpper;
    }
}
