package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production;

public class VelocityRamping {

    public double MAX_VELOCITY;
    public double acceleration;

    public VelocityRamping(double maxVel){
        this.MAX_VELOCITY = maxVel;
    }

    public void solve(double distance, double time){
        double distMax = Math.max((2 * distance) / time, MAX_VELOCITY);
        acceleration = distMax / (time - (distance / distMax));
    }
}
