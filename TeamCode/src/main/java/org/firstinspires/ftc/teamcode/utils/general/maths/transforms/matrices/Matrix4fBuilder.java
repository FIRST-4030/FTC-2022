package org.firstinspires.ftc.teamcode.utils.general.maths.transforms.matrices;

import org.firstinspires.ftc.teamcode.utils.general.maths.transforms.vectors.Vector3f;
import org.firstinspires.ftc.teamcode.utils.general.maths.transforms.vectors.Vector4f;

public class Matrix4fBuilder {

    /**
     * Builds a translation matrix that transforms the X, Y, Z by the forth column, vector t
     * @param translation
     * @return
     */
    public Matrix4f buildTranslation(Vector3f translation){
        return new Matrix4f(
                new float[]{1, 0, 0, translation.getX(),
                            0, 1, 0, translation.getY(),
                            0, 0, 1, translation.getZ(),
                            0, 0, 0,                  1});
    }

    /**
     * Builds a scaling matrix that scales the X, Y, Z by a single factor
     * @param factor
     * @return
     */
    public Matrix4f buildDilation(float factor){
        return new Matrix4f(
                new float[]{factor,      0,      0, 1,
                                 0, factor,      0, 1,
                                 0,      0, factor, 1,
                                 0,      0,      0, 1});
    }

    /**
     * Builds a scaling matrix that scales the X, Y, Z individually
     * @param xFactor
     * @param yFactor
     * @param zFactor
     * @return
     */
    public Matrix4f buildDialation(float xFactor, float yFactor, float zFactor){
        return new Matrix4f(
                new float[]{xFactor, 0, 0, 1,
                            0, yFactor, 0, 1,
                            0, 0, zFactor, 1,
                            0, 0,       0, 1}
        );
    }

    /**
     * Builds a rotation matrix on the X axis ('Pitch')
     * @param angle_deg
     * @return
     */
    public static Matrix4f buildRotX(double angle_deg){
        float cos = (float) Math.cos(Math.toRadians(angle_deg));
        float sin = (float) Math.sin(Math.toRadians(angle_deg));

        return new Matrix4f(
                new float[]{1,   0,    0, 0,
                            0, cos, -sin, 0,
                            0, sin,  cos, 0,
                            0,   0,    0, 1});
    }

    /**
     * Builds a rotation matrix on the Y axis ('Yaw')
     * @param angle_deg
     * @return
     */
    public static Matrix4f buildRotY(double angle_deg){
        float cos = (float) Math.cos(Math.toRadians(angle_deg));
        float sin = (float) Math.sin(Math.toRadians(angle_deg));

        return new Matrix4f(
                new float[]{ cos, 0, sin, 0,
                               0, 1,   0, 0,
                            -sin, 0, cos, 0,
                               0, 0,   0, 1});
    }

    /**
     * Builds a rotation matrix on the Z axis ('Roll')
     * @param angle_deg
     * @return
     */
    public static Matrix4f buildRotZ(double angle_deg){
        float cos = (float) Math.cos(Math.toRadians(angle_deg));
        float sin = (float) Math.sin(Math.toRadians(angle_deg));

        return new Matrix4f(
                new float[]{cos, -sin, 0, 0,
                            sin,  cos, 0, 0,
                              0,    0, 0, 0,
                              0,    0, 0, 1});
    }


    /**
     * build a general rotation using Euler angles that describe the rotation of an object
     * @param pitch_deg
     * @param yaw_deg
     * @param roll_deg
     * @return
     */
    public static Matrix4f buildGenRot(double pitch_deg, double yaw_deg, double roll_deg){
        //handles yaw rot
        float cosa = (float) Math.cos(Math.toRadians(yaw_deg));
        float sina = (float) Math.sin(Math.toRadians(yaw_deg));

        //handles pitch rot
        float cosr = (float) Math.cos(Math.toRadians(pitch_deg));
        float sinr = (float) Math.sin(Math.toRadians(pitch_deg));

        //handles roll rot
        float cosb = (float) Math.cos(Math.toRadians(roll_deg));
        float sinb = (float) Math.sin(Math.toRadians(roll_deg));

        return new Matrix4f(
                new float[]{cosa * cosb, cosa * sinb * sinr - sina * cosr, cosa * sinb * cosr + sina * sinr, 0,
                            sina * cosb, sina * sinb * sinr + cosa * cosr, sina * sinb * cosr - cosa * sinr, 0,
                                  -sinb,                      cosb * sinr,                      cosb * cosr, 0,
                                      0,                                0,                                0, 1});
    }

    /**
     * builds a matrix that combines the general rotation matrix and add a translation vector
     * @param pitch_deg
     * @param yaw_deg
     * @param roll_deg
     * @param translation
     * @return
     */
    public static Matrix4f buildTransform(double pitch_deg, double yaw_deg, double roll_deg, Vector3f translation){
        float[] output = buildGenRot(pitch_deg, yaw_deg, roll_deg).getAsFloatArray();
        output[3] = translation.getX();
        output[7] = translation.getY();
        output[11] = translation.getZ();
        return new Matrix4f(output);
    }

    /**
     * Build a camera matrix with the normals of the described camera with the translation
     * @param forward
     * @param right
     * @param up
     * @param translation
     * @return
     */
    public static Matrix4f buildCamera(Vector3f forward, Vector3f right, Vector3f up, Vector3f translation){
        return new Matrix4f(
                new float[]{forward.getX(), forward.getY(), forward.getZ(), -Vector3f.dot(forward, translation),
                              right.getX(),   right.getY(),   right.getZ(),   -Vector3f.dot(right, translation),
                                 up.getX(),      up.getY(),      up.getZ(),      -Vector3f.dot(up, translation),
                                         0,              0,              0,                                 1});
    }

    /**
     * Build a camera matrix with the euler angles of the described camera with the translation
     * @param pitch_deg
     * @param yaw_deg
     * @param roll_deg
     * @param translation
     * @return
     */
    public static Matrix4f buildCamera(double pitch_deg, double yaw_deg, double roll_deg, Vector3f translation){
        Matrix4f rot = buildGenRot(pitch_deg, yaw_deg, roll_deg);
        Vector3f forward = rot.matMul(new Vector4f(0, 0, 1, 1)).getAsVec3f();
        Vector3f right = rot.matMul(new Vector4f(1, 0, 0, 1)).getAsVec3f();
        Vector3f up = rot.matMul(new Vector4f(0, 1, 0, 1)).getAsVec3f();

        return new Matrix4f(
                new float[]{forward.getX(), forward.getY(), forward.getZ(), -Vector3f.dot(forward, translation),
                        right.getX(),   right.getY(),   right.getZ(),   -Vector3f.dot(right, translation),
                        up.getX(),      up.getY(),      up.getZ(),      -Vector3f.dot(up, translation),
                        0,              0,              0,                                 1});
    }
}
