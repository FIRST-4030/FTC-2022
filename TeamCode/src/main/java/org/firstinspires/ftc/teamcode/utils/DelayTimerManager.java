package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayTimerManager {

    private static DelayTimerManager instance = null;

    private static ElapsedTime eTime = new ElapsedTime();

    public static void makeInstance(){
        if (DelayTimerManager.instance == null){
            DelayTimerManager.instance = new DelayTimerManager();
        } else {
            throw new IllegalStateException("Already Instantiated! ");
        }
    }

    public static DelayTimerManager getInstance(){
        return DelayTimerManager.instance;
    }

    public static void startGlobalTimer(){
        DelayTimerManager.eTime.startTime();
    }

    public static void resetGlobalTimer(){
        DelayTimerManager.eTime.reset();
    }

    public static double getCurrentGlobalTime(){
        return DelayTimerManager.eTime.time();
    }

    /**
     * This class is a timer that is based on a global ElapsedTime, DelayTimeManager.makeInstance() must be called before any instance of this is made;
     */
    public class DelayTimer {

        private double startTime = 0d, stopTime = 0d, elapsedTime = 0d;

        public DelayTimer(){}

        /**
         * This method "starts" the timer by setting this.startTime to the current time that the method is called
         */
        public void start(){
            this.startTime = DelayTimerManager.getCurrentGlobalTime();
        }

        /**
         * This method "stops" the timer by setting this.stopTime to the current time the method is called
         * It will also set this.elapsedTime
         */
        public void stop(){
            this.stopTime = DelayTimerManager.getCurrentGlobalTime();
            this.elapsedTime = this.stopTime - this.startTime;
        }

        /**
         * This method resets all the variables (this.startTime; this.stopTime; this.elapsedTime;) to 0
         */
        public void reset(){
            this.startTime = 0d;
            this.stopTime = 0d;
            this.elapsedTime = 0d;
        }

        /**
         * This method is to return the current time from the start, not the current global time
         * @return DelayTimerManager.getCurrentGlobalTime() - this.startTime;
         */
        public double getCurrentTime(){
            return DelayTimerManager.getCurrentGlobalTime() - this.startTime;
        }

        /**
         * This method gives the time from the start to end, this value is 0 by default
         * @return this.elapsedTime;
         */
        public double getElapsedTime(){
            return this.elapsedTime;
        }
    }
}
