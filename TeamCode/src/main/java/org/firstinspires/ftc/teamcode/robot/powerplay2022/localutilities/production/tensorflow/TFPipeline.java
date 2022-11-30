package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.tensorflow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.ext.TFBoundingBox;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.main.TFODBase;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain.CameraLens;

import java.util.HashMap;

public class TFPipeline {

    public enum PipelineState{
        SCANNING,
        SORTING,
        IDLE
    }

    public TFODBase tfodBase;

    public HashMap<String, TFBoundingBox> closestBB;

    public TFBoundingBox masterCone, masterJunction;

    public PipelineState status;

    public boolean hasScanned;

    public double[] C270 = CameraLens.C270_FOV;

    public double maxSizePX = 1280, widthCone = 10, widthPole = 2.5, hFOV = C270[0];

    public float findDepth(double absWidth, double bbPx){
        return (float) ((absWidth/(2* Math.sin((hFOV*bbPx)/(2*maxSizePX)))) - (absWidth/2));
    }

    public TFPipeline(HardwareMap hardwareMap, String cameraName, String tensorflowModel, String[] tensorflowLabels){
        this.tfodBase = new TFODBase(hardwareMap, cameraName, tensorflowModel, tensorflowLabels);
        this.closestBB = new HashMap<>();

        for (String label: tensorflowLabels) {
            closestBB.put(label, new TFBoundingBox());
        }

        this.status = PipelineState.IDLE;
    }

    public void init(){
        tfodBase.opInit();
    }

    public void scan(){
        status = PipelineState.SCANNING;
        tfodBase.scan();
        status = PipelineState.IDLE;
    }

    public void sort(){
        status = PipelineState.SORTING;
        TFBoundingBox cachedMin = new TFBoundingBox();
        float depth;

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Blue Cone")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthCone, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Blue Cone", cachedMin);
        cachedMin = new TFBoundingBox();

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Red Cone")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthCone, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Red Cone", cachedMin);
        cachedMin = new TFBoundingBox();

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Junction Top")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthPole, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("JunctionTop")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthPole, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Junction Top", cachedMin);
        status = PipelineState.IDLE;

        if(closestBB.get("Blue Cone").estimatedDepth > 30){
            //Do a thing
        }
        if(closestBB.get("Red Cone").estimatedDepth > 30){
            //Do a thing
        }
        if(closestBB.get("JunctionTop").estimatedDepth > 30){
            //Do a thing
        }
    }

    public double correctionAngle(TFBoundingBox box){
        double x = box.getCenterPoint().x;
        x -= (maxSizePX/2);
        x = (2*x/maxSizePX) * hFOV/2;
        x *= Math.PI/3;
        return x;
    }

    public void update(){
        scan();
        sort();
    }
}
