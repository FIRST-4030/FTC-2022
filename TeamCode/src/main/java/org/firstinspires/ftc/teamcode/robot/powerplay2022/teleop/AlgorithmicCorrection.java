package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

public class AlgorithmicCorrection {

    public interface InterpolationAlgorithm{
        double process(double scalar);
    }

    public static class CustomizableRELU implements InterpolationAlgorithm{

        private double slope;
        private double yIntercept;

        public CustomizableRELU(double slope, double yIntercept){
            this.slope = slope;
            this.yIntercept = yIntercept;
        }

        @Override
        public double process(double scalar) {
            return Math.max(0, scalar * slope + yIntercept);
        }
    }

    private double output;
    private Matrix2d actualRotation;
    private Matrix2d targetRotation;

    private Vector2d headingVector;
    private Vector2d targetVector;
    private Vector2d perpendicularTargetVector;

    private double targetDistance; //a scalar value based on half of the circumference
    private double correctionSign;
    private InterpolationAlgorithm interpolationAlgorithm;

    public AlgorithmicCorrection(){
        this.interpolationAlgorithm = new CustomizableRELU(1, 1);
    }

    public AlgorithmicCorrection(InterpolationAlgorithm algorithm){
        this.interpolationAlgorithm = algorithm;
    }

    protected void init(){
        this.output = 0;
        this.actualRotation = new Matrix2d();
        this.targetRotation = new Matrix2d();

        this.headingVector = new Vector2d(1, 0);
        this.targetVector = new Vector2d(1, 0);
        this.perpendicularTargetVector = new Vector2d(1, 0);

        this.targetDistance = 0;
        this.correctionSign = 1;
    }

    public void update(double actualAngle, double targetAngle, boolean normalize){
        //find the rotation matrices for the angles passed in
        targetRotation = Matrix2d.makeRotation(targetAngle);
        actualRotation = Matrix2d.makeRotation(actualAngle);

        //multiply prerequisite vectors to be used later on
        targetVector = targetRotation.times(new Vector2d(0, 1));
        perpendicularTargetVector = new Vector2d(targetVector.y, -targetVector.x);
        headingVector = actualRotation.times(new Vector2d(0, 1));

        //normalize if true to make sure the length of the vectors are 1
        //if on, it might waste cpu cycles to make sure
        if (normalize) {
            targetVector.normalize();
            perpendicularTargetVector.normalize();
            headingVector.normalize();
        }

        //since the dot product is [-1, 1], we shift it to [0, 2] then divide to normalize it to [0, 1]
        //this distance represents the shortest from the target (on a unit circle)
        targetDistance = (targetVector.unaryMinus().times(headingVector) + 1) / 2;

        //finds if the vector is to the right or left (robot's space; not world space)
        //in-line if is for resolving exact value conditions even though they are rare
        correctionSign = Math.signum(headingVector.times(perpendicularTargetVector)) == 0? 1: -Math.signum(headingVector.times(perpendicularTargetVector));

        //input the scalar into the interpolation and multiply by the correction sign
        output = interpolationAlgorithm.process(1 - targetDistance) * correctionSign;
    }

    public double getOutput(){
        return output;
    }

    public void log(Telemetry telemetry){
        //logs the general data of correction, stored heading and target, shortest path scalar, correction sign... etc.
        telemetry.addData("Correction Output: ", output);
        telemetry.addData("Heading Vector: ", headingVector);
        telemetry.addData("Target Vector: ", targetVector);
        telemetry.addData("Target Perpendicular Vector: ", perpendicularTargetVector);
        telemetry.addData("Distance Scalar: ", targetDistance + "π");
        telemetry.addData("Correction Sign: ", correctionSign);
    }
}
