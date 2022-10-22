package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class TFODBase{

    public enum STATE{
        SCANNING,
        IDLE
    }

    public static final String VUFORIA_KEY = "AV9rwXT/////AAABma+8TAirNkVYosxu9qv0Uz051FVEjKU+nkH+MaIvGuHMijrdgoZYBZwCW2aG8P3+eZecZZPq9UKsZiTHAg73h09NT48122Ui10c8DsPe0Tx5Af6VaBklR898w8xCTdOUa7AlBEOa4KfWX6zDngegeZT5hBLfJKE1tiDmYhJezVDlITIh7SHBv0xBvoQuXhemlzL/OmjrnLuWoKVVW0kLanImI7yra+L8eOCLLp1BBD/Iaq2irZCdvgziZPnMLeTUEO9XUbuW8txq9i51anvlwY8yvMXLvIenNC1xg4KFhMmFzZ8xnpx4nWZZtyRBxaDU99aXm7cQgkVP0VD/eBIDYN4AcB0/Pa7V376m6tRJ5UZh";

    public String tensorflowModel;
    public String cameraName;
    public String[] tensorflowLabels;

    public HashMap<String, ArrayList<Recognition>> recognitions;

    public boolean isDone = true;
    public boolean isInitialized = false;

    public VuforiaLocalizer vuforiaLocal;
    public TFObjectDetector tensorflow;
    public HardwareMap hardwareMap;
    public STATE state;

    public TFODBase(HardwareMap hardwareMap, String cameraName, String tensorflowModel, String[] tensorflowLabels){
        this.cameraName = cameraName;
        this.tensorflowModel = tensorflowModel;
        this.tensorflowLabels = tensorflowLabels;
        this.hardwareMap = hardwareMap;
    }

    protected void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, cameraName);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = null;

        vuforiaLocal = ClassFactory.getInstance().createVuforia(parameters);
    }

    protected void initTensorFlow(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.useObjectTracker = true;
        tfodParameters.inputSize = 320;
        tensorflow = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforiaLocal);
        tensorflow.loadModelFromAsset(tensorflowModel, tensorflowLabels);
    }

    public void opInit() {
        initVuforia();
        initTensorFlow();
        isInitialized = true;
    }

    public void scan(){
        state = STATE.SCANNING;
        isDone = false;

        if (tensorflow == null) return; //abort if tensorflow is null

        List<Recognition> tfOutput = tensorflow.getRecognitions();

        if (recognitions == null) {
            clearRecognitions();
            return;
        } //abort if tensorflow doesn't recognize anything

        clearRecognitions();

        for (Recognition output: tfOutput) {
            if (!checkLabels(output.getLabel())) {
                this.recognitions.put(output.getLabel(), new ArrayList<>());
            }

            this.recognitions.get(output.getLabel()).add(output);
        }

        state = STATE.IDLE;
        isDone = true;
    }

    public void clearRecognitions(){
        for (String label: tensorflowLabels) {
            this.recognitions.put(label, new ArrayList<>());
        }
    }

    public boolean checkLabels(String in){
        for (String s: tensorflowLabels) {
            if (s.equals(in)){
                return true;
            }
        }

        return false;
    }

    public ArrayList<Recognition> filterRecognitions(String query){
        return this.recognitions.get(query);
    }

    public ArrayList<Recognition>[] filterRecognitions(String... queries){
        if (!(queries.length > recognitions.size())) throw new IllegalArgumentException("Filters EXCEED hashmap length; doesn't exist");
        ArrayList<Recognition>[] output = new ArrayList[queries.length];

        for (int i = 0; i < output.length; i++) {
            output[i] = filterRecognitions(queries[i]);
        }

        return output;
    }

    public void opStop() {
        tensorflow.shutdown();
    }
}
