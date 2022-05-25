package org.firstinspires.ftc.teamcode.robot.rrImpl.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Pose2dRecorder {

    private List<Pose2d> recordedPoses;
    private Pose2d lastPose = null;

    public Pose2dRecorder(){
        this.recordedPoses = new ArrayList<>();
    }

    public void record(Pose2d pose2d){
        if ((lastPose == null) || (!lastPose.equals(pose2d))) {
            recordedPoses.add(pose2d);
            lastPose = pose2d;
        }
    }

    public void recordAll(Pose2d[] pose2ds){
        for (Pose2d pose : pose2ds){
            if ((lastPose == null) || (!lastPose.equals(pose))) {
                recordedPoses.add(pose);
            }
        }
    }

    public void removeAt(int index){
        recordedPoses.remove(index);
    }

    public void removeLatest(){
        recordedPoses.remove(recordedPoses.size() - 1);
    }

    public void removeAll(){
        recordedPoses.clear();
        lastPose = null;
    }

    public void replaceAt(int index, Pose2d newPath){
        this.recordedPoses.set(index, newPath);
    }

    public void replaceIn(int offset, Pose2d[] paths){
        for (int i = 0; i < paths.length; i++){
            this.recordedPoses.set(i + offset, paths[i]);
        }
    }

    public List<Pose2d> getAsList(){
        return this.recordedPoses;
    }

    public Pose2d[] getAsArray(){
        Pose2d[] output = new Pose2d[this.recordedPoses.size()];
        for (int i = 0; i < this.recordedPoses.size(); i++){
            output[i] = this.recordedPoses.get(i);
        }
        return output;
    }
}
