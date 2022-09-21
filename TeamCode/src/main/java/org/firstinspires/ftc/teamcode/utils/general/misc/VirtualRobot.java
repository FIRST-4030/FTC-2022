package org.firstinspires.ftc.teamcode.utils.general.misc;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;

public class VirtualRobot {
    public Vector3d position;
    public Vector3d velocity;
    public Vector3d acceleration;
    public double headingAngle;

    private Vector2d xAxis;
    private Vector2d yAxis;
    private Vector2d translation;
    private Matrix3d transform;

    public VirtualRobot(){
        this.position = new Vector3d();
        this.velocity = new Vector3d();
        this.acceleration = new Vector3d();

        this.headingAngle = 0;
    }

    public void update(){
        Matrix2d rot = Matrix2d.makeRotation(headingAngle);
        xAxis = rot.times(new Vector2d(1, 0));
        yAxis = rot.times(new Vector2d(0, 1));
        updateMatrix();
    }

    private void updateMatrix(){
        transform = Matrix3d.makeAffineRotation(headingAngle);
        transform.matrix[0][2] = translation.x;
        transform.matrix[1][2] = translation.y;
    }
}
