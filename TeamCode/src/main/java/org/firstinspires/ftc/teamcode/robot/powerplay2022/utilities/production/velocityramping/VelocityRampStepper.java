package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.velocityramping;

import java.util.ArrayList;

public class VelocityRampStepper {
    
    public static class VelocityRampEquation{
        private double accel, deceleration, maxAccel;
        private double xInt;

        public VelocityRampEquation(double maxAccel, double accel, double time){
            this.maxAccel = maxAccel;
            this.accel = accel;
            this.deceleration = -accel;
            this.xInt = time;
        }

        public double solve(double time){
            return Math.min(Math.min(accel * time, deceleration * (time - xInt)), maxAccel);
        }
    }
    
    public VelocityRamping accelGenerator;
    
    private ArrayList<VelocityRampEquation> accelEquations;
    public double estimatedElapsedTime, stateElapsedTime;
    public int currentRamp;
    
    public VelocityRampStepper(){
        this.accelGenerator = new VelocityRamping(1);
        init();
    }

    public VelocityRampStepper(VelocityRamping velocityRamping){
        this.accelGenerator = velocityRamping;
        init();
    }

    private void init(){
        accelEquations = new ArrayList<>(1);
        estimatedElapsedTime = 0;
        stateElapsedTime = 0;
        currentRamp = 0;
    }

    public void addRamp(double distance, double time){
        estimatedElapsedTime += time;
        accelEquations.add(
                new VelocityRampEquation(
                        accelGenerator.MAX_VELOCITY,
                        accelGenerator.solve(distance, time).acceleration,
                        time
                        )
        );
    }

    public void addRamps(double[] distances, double[] times){
        for (int i = 0; i < Math.min(distances.length, times.length); i++) {
            estimatedElapsedTime += times[i];
            accelEquations.add(
                    new VelocityRampEquation(
                            accelGenerator.MAX_VELOCITY,
                            accelGenerator.solve(distances[i], times[i]).acceleration,
                            times[i]
                    )
            );
        }
    }

    public double update(double deltaTime){
        stateElapsedTime += deltaTime;
        if (stateElapsedTime > accelEquations.get(currentRamp).xInt){
            currentRamp += 1; //increment current ramp index by 1
            stateElapsedTime = 0; //reset current ramp's time in state
        }
        return accelEquations.get(currentRamp).solve(stateElapsedTime) / accelGenerator.MAX_VELOCITY;
    }
}
