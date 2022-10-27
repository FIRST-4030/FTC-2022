package org.firstinspires.ftc.teamcode.utils.general;

public abstract class RunOnce implements Runnable{

    private boolean latch = false;

    public void update(){
        if (!latch){
            run();
            latch = true;
        }
    }

    public abstract void run();
}
