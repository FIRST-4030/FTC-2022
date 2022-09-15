package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.polyorder.integration;

import com.icaras84.extrautilslib.core.timer.EULClock;

public abstract class EULIntegrator {
    protected EULClock timer;

    public void init(){
        this.timer = new EULClock();
        internalInit();
        this.timer.start();
    }

    protected abstract void internalInit();
    protected abstract void updateVar();
    public abstract void update(double time);
}
