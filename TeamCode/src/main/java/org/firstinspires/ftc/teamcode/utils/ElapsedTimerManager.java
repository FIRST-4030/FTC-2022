package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class ElapsedTimerManager {

    private ArrayList<ElapsedTime> timerList;
    private ArrayList<Boolean> timerToggleList;
    private ArrayList<String> timerName;

    public ElapsedTimerManager(){
        this.timerList = new ArrayList<>();
        this.timerToggleList = new ArrayList<>();
        this.timerName = new ArrayList<>();
    }

    public void addTimer(String name, ElapsedTime.Resolution resolution){
        if (!this.timerName.contains(name)){
            int idx = this.timerName.indexOf(name);
            this.timerName.add(idx, name);
            this.timerList.add(this.timerName.indexOf(name), new ElapsedTime(resolution));
            this.timerToggleList.add(idx, false);
        }
    }

    public ElapsedTime findTimer(String name){
        if (this.timerName.contains(name)){
            int idx = this.timerName.indexOf(name);
            return this.timerList.get(idx);
        } else {
            throw new IllegalArgumentException("Not Registered!");
        }
    }

    public void start(String name){
        if (this.timerName.contains(name)){
            int idx = this.timerName.indexOf(name);
            this.timerList.get(idx).startTime();
            this.timerToggleList.set(idx, true);
        } else {
            throw new IllegalArgumentException("No such timer exists!");
        }
    }

    public void reset(String name){
        if (this.timerName.contains(name)){
            int idx = this.timerName.indexOf(name);
            this.timerList.get(idx).reset();
            this.timerToggleList.set(idx, false);
        } else {
            throw new IllegalArgumentException("No such timer exists!");
        }
    }

    public void startAll(){
        for (int i = 0; 0 < this.timerName.size(); i++){
            this.timerList.get(i).startTime();
            this.timerToggleList.set(i, true);
        }
    }

    public void resetAll(){
        for (int i = 0; 0 < this.timerName.size(); i++){
            this.timerList.get(i).reset();
            this.timerToggleList.set(i, false);
        }
    }
}
