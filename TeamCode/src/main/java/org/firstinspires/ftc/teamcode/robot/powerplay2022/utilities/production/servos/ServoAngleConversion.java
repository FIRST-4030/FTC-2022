package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos;

import java.util.Arrays;

public class ServoAngleConversion {

    public enum UNIT{
        RADIANS,
        DEGREES
    }

    private double min, max;
    private UNIT unit;

    public ServoAngleConversion(double servoMin, double servoMax, UNIT unit){
        double[] temp = new double[]{min, max};
        Arrays.sort(temp);
        this.min = temp[0];
        if (unit == UNIT.RADIANS) this.min *= ((double)3/2) * Math.PI;
        else if (unit == UNIT.DEGREES) this.min *= 270;

        if (unit == UNIT.RADIANS) this.max *= ((double)3/2) * Math.PI;
        else if (unit == UNIT.DEGREES) this.max *= 270;

        this.max = temp[1];
        this.unit = unit;
    }


    public double angle2Scalar(double measurement){


        return 0.0;
    }
}
