package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.tensorflow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.ext.TFBoundingBox;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.main.TFODBase;

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
            if (cachedMin.estimatedDepth > (depth = boundingBox.getDepthEstimate(masterCone, TFBoundingBox.DEPTH_PRECISION.LOW)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Blue Cone", cachedMin);
        cachedMin = new TFBoundingBox();

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Red Cone")) {
            if (cachedMin.estimatedDepth > (depth = boundingBox.getDepthEstimate(masterCone, TFBoundingBox.DEPTH_PRECISION.LOW)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Red Cone", cachedMin);
        cachedMin = new TFBoundingBox();

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Junction Top")) {
            if (cachedMin.estimatedDepth > (depth = boundingBox.getDepthEstimate(masterJunction, TFBoundingBox.DEPTH_PRECISION.LOW)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("JunctionTop")) {
            if (cachedMin.estimatedDepth > (depth = boundingBox.getDepthEstimate(masterJunction, TFBoundingBox.DEPTH_PRECISION.LOW)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Junction Top", cachedMin);
        status = PipelineState.IDLE;
    }

    public void update(){
        scan();
        sort();
    }
}
