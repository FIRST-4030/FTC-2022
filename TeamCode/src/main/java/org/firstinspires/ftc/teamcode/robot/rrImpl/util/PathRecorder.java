package org.firstinspires.ftc.teamcode.robot.rrImpl.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.ArrayList;
import java.util.List;

public class PathRecorder {

    private List<Path> recordedPaths;
    private Path lastPath = null;

    public PathRecorder(){
        this.recordedPaths = new ArrayList<>();
    }

    public void record(Path path){
        if ((lastPath == null) || (!lastPath.equals(path))) {
            recordedPaths.add(path);
            lastPath = path;
        }
    }

    public void recordAll(Path[] paths){
        for (Path path : paths){
            if ((lastPath == null) || (!lastPath.equals(path))) {
                recordedPaths.add(path);
                lastPath = path;
            }
        }
    }

    public void removeAt(int index){
        recordedPaths.remove(index);
    }

    public void removeLatest(){
        recordedPaths.remove(recordedPaths.size() - 1);
    }

    public void removeAll(){
        recordedPaths.clear();
        lastPath = null;
    }

    public void replaceAt(int index, Path newPath){
        this.recordedPaths.set(index, newPath);
    }

    public void replaceIn(int offset, Path[] paths){
        for (int i = 0; i < paths.length; i++){
            this.recordedPaths.set(i + offset, paths[i]);
        }
    }

    public List<Path> getAsList(){
        return this.recordedPaths;
    }

    public Path[] getAsArray(){
        Path[] output = new Path[this.recordedPaths.size()];
        for (int i = 0; i < this.recordedPaths.size(); i++){
            output[i] = this.recordedPaths.get(i);
        }
        return output;
    }
}
