package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

public class VectorAxis {

    public Vector2d xAxis, yAxis;

    public VectorAxis(Vector2d vector, boolean isForward){
        if (isForward){
            this.yAxis = vector;
            this.xAxis = new Vector2d(this.yAxis.y, -this.yAxis.x);
        } else {
            this.xAxis = vector;
            this.yAxis = new Vector2d(-this.xAxis.y, this.xAxis.x);
        }
    }
}
