package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos;

import java.util.Arrays;

public class ServoAngleConversion {

    public enum ANGLE_UNIT{
        RADIANS,
        DEGREES,
        TURNS
    }

    private double min, max, angleMulti, limitedAngle, output;
    private boolean outOfRange;
    private ANGLE_UNIT unit;

    public ServoAngleConversion(double min, double max){
        this.angleMulti = 2 * Math.PI;

        double[] temp = new double[]{min, max};
        Arrays.sort(temp);
        this.min = temp[0] / this.angleMulti;
        this.max = temp[1] / this.angleMulti;
        this.unit = ANGLE_UNIT.RADIANS;

        this.limitedAngle = 0.75 * this.angleMulti;
    }


    public ServoAngleConversion(double min, double max, ANGLE_UNIT unit){
        switch (unit){
            case TURNS:
                this.angleMulti = 1;
                break;
            case DEGREES:
                this.angleMulti = 360;
                break;
            case RADIANS:
                this.angleMulti = 2 * Math.PI;
                break;
        }

        double[] temp = new double[]{min, max};
        Arrays.sort(temp);
        this.min = temp[0] / this.angleMulti;
        this.max = temp[1] / this.angleMulti;
        this.unit = unit;

        this.limitedAngle = 0.75 * this.angleMulti;
    }


    public void angle2Scalar(double measurement){
        output = angleMulti * (measurement / limitedAngle);
        outOfRange = min > output || max < output;
    }

    public void scalar2Angle(double scalar){
        output = limitedAngle * (scalar / angleMulti);
        outOfRange = min > output || max < output;
    }

    public double getOutput(){
        return output;
    }

    public boolean isOutOfRange(){
        return outOfRange;
    }
}
