package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import org.firstinspires.ftc.teamcode.pathfinder.control.PathFinderDrive;

import java.util.Vector;

public class PFPath {

    private PathFinderDrive drive;
    protected Vector<float[]> encoderValues;
    protected Vector<PFPose2d> poseLookup;
    protected int idx;

    public PFPath(PathFinderDrive drive, PFPose2d initialPose){
        this.encoderValues = new Vector<>();
        this.poseLookup = new Vector<>();
        this.idx = 0;

        this.poseLookup.add(initialPose);
        this.drive = drive;
    }

    public void updatePathProgress(float... realEncoderValues){
        float[] previousValues = new float[realEncoderValues.length];
        float[] targetValues = new float[previousValues.length];
        float averageProgress = 0;

        for (int i = 0; i < realEncoderValues.length; i++) {
            previousValues[i] = idx > 0 ? encoderValues.get(i)[idx - 1] : 0;
            targetValues[i] = encoderValues.get(i)[idx];

            averageProgress += (realEncoderValues[i] - previousValues[i]) / (targetValues[i] - previousValues[i]);
        }

        averageProgress /= realEncoderValues.length;

        if (averageProgress >= 1.0f) idx++;
    }

    public Vector<PFPose2d> getPoseLookup(){
        return this.poseLookup;
    }

    public void build(){
        this.drive.buildPath(this);
    }

    public void setEncoderValues(Vector<float[]> nValues) {
        this.encoderValues = nValues;
    }

    public float[] getCurrentEncoderValues(){
        return this.encoderValues.get(idx);
    }
}
