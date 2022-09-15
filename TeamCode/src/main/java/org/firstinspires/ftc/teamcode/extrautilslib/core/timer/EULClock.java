package org.firstinspires.ftc.teamcode.extrautilslib.core.timer;

public class EULClock {

    //NANOSECONDS to X conversions coefficients
    public static final double NANO2MS = 1e-6;
    public static final double NANO2SEC = 1e-9;

    //MILLISECONDS to X conversions coefficients
    public static final double MS2NANO = 1e6;
    public static final double MS2SEC = 1e-3;

    //SECONDS to X conversion coefficients
    public static final double SEC2NANO = 1e9;
    public static final double SEC2MS = 1e3;

    private long startTime = 0, deltaTime = 0, stopTime = 0;
    private boolean stopped = true;

    public EULClock(){}

    public void start(){
        this.stopped = false;
        this.startTime = System.nanoTime();
        this.stopTime = this.startTime;
    }

    public void stop(){
        this.stopped = true;
        this.stopTime = System.nanoTime();
        this.deltaTime = this.stopTime - this.startTime;
    }

    public long getStartTime(){return this.startTime;}
    public long getDeltaTime(){return this.stopped ? this.deltaTime : getElapsedTime();}
    public long getStopTime(){return this.stopped ? this.stopTime : getElapsedTime();}
    public long getElapsedTime(){return System.nanoTime() - this.startTime;}
    public boolean isStopped(){return this.stopped;}

    public void waitFor(long ms){
        double ns = ms * MS2NANO;
        long start = System.nanoTime();
        while (System.nanoTime() - start < ns){}
    }
}
