package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine;

import java.util.ArrayList;
import java.util.Arrays;

public class OpState {
    public ArrayList<Runnable> asyncRunnable;

    public OpState(){
        this.asyncRunnable = new ArrayList<>();
    }

    public OpState(Runnable... runnable) {
        this.asyncRunnable = new ArrayList<>();
        this.asyncRunnable.addAll(Arrays.asList(runnable));
    }

    public void runAll(){
        for (Runnable runnable: asyncRunnable) {
            runnable.run();
        }
    }
}
