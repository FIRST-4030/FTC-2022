package org.firstinspires.ftc.teamcode.extrautilslib.samples;

import com.icaras84.extrautilslib.core.maths.EULMathEx;
import com.icaras84.extrautilslib.core.maths.matrices.Matrix3d;
import com.icaras84.extrautilslib.core.maths.matrices.Matrix4d;
import com.icaras84.extrautilslib.core.maths.vectors.EULVectorUtils;
import com.icaras84.extrautilslib.core.maths.vectors.Vector2d;
import com.icaras84.extrautilslib.core.maths.vectors.Vector3d;
import com.icaras84.extrautilslib.core.maths.vectors.Vector4d;
import com.icaras84.extrautilslib.core.maths.vectors.quaternions.Quaternion;

import java.util.Arrays;

public class MathTest {

    public static void main(String[] args) {
        /*
        Matrix3d m1 = new Matrix3d(new double[][]{
                {1, 2, 3},
                {4, 5, 6},
                {7, 8, 9}
        });
        Matrix3d m2 = new Matrix3d(new double[][]{
                {0.5, 0.1, 0.4},
                {0.8, 0.2, 0.0},
                {0.3, 0.3, 0.4}
        });

        System.out.println("Test: m1 * m2 ; m2 * m1");
        System.out.println(m1.times(m2));
        System.out.println(m2.times(m1));

        */

        /*
        Matrix4d qRotation, nRotation;
        Vector4d[] testVectors = new Vector4d[]{
                new Vector4d(1, 0, 0, 1), //sanity check
                new Vector4d(0, 1, 0, 1), //y-axis aligned
                new Vector4d(0, 0, 1, 1), //z-axis aligned
                new Vector4d(1, 1, 1, 1)
        };

        for (int i = 0; i < 16; i++) {
            double angle = i * 22.5;
            qRotation = (new Quaternion(new Vector3d(0, 1, 0), angle * EULMathEx.DEG2RAD)).getAsPointRotationMatrix();
            nRotation = Matrix4d.makeAffineRotation(EULMathEx.Axis.AXIS_Y, angle * EULMathEx.DEG2RAD);

            //for (Vector4d v : testVectors) {
                System.out.println("Input vector: " + testVectors[3]);
                System.out.println("Quaternion result: " + qRotation.times(testVectors[3]));
                System.out.println("Normal rotation Matrix result: " + nRotation.times(testVectors[3]));
                System.out.println();
            //}
        }

         */

        Vector4d a = new Vector4d(0, 0, 0, 0);
        Vector4d b = new Vector4d(4, 3, 2, 1);
        System.out.println(EULVectorUtils.lerp(a, b, 0.5));
    }
}
