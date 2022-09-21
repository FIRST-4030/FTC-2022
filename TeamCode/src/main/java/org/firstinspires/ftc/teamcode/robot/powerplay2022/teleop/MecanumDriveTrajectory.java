package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULArrays;

import java.util.Stack;

public class MecanumDriveTrajectory {

    private Stack<MecanumDriveState> cache;
    private Stack<MecanumDriveState> stateStack;
    private Stack<MecanumDriveState> copy;
    private CustomMecanumDrive drive;

    public MecanumDriveTrajectory(CustomMecanumDrive drive){
        this.stateStack = new Stack<>();
        this.copy = new Stack<>();
        this.drive = drive;
    }

    public void build(){
        for (int i = 0; i < stateStack.size(); i++){
            this.copy.push(stateStack.get(i));
        }
    }

    public MecanumDriveState pop(){
        return copy.pop();
    }

    public MecanumDriveState peek(){
        return copy.peek();
    }

    public Stack<MecanumDriveState> getCurrentStateStack(){
        return copy;
    }

    public boolean isCurrentStateStackEmpty(){
        return copy.isEmpty();
    }

    public MecanumDriveTrajectory forward(double duration){
        cache.push(new MecanumDriveState("FORWARD", () -> {
            //drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = 1;
            //drive.virtualJoystick.z = 0;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory backward(double duration){
        cache.push(new MecanumDriveState("BACKWARDS", () -> {
            //drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = -1;
            //drive.virtualJoystick.z = 0;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory turnRight(double duration){
        cache.push(new MecanumDriveState("TURN_RIGHT", () -> {
            //drive.virtualJoystick.x = 0;
            //drive.virtualJoystick.y = 0;
            drive.virtualJoystick.z = 1;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory turnLeft(double duration){
        cache.push(new MecanumDriveState("TURN_LEFT", () -> {
            drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = 0;
            drive.virtualJoystick.z = -1;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }
}
