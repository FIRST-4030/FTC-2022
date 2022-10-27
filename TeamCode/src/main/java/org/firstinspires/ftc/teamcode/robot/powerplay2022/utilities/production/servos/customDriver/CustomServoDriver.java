package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.customDriver;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULArrays;

import java.util.Stack;

public class CustomServoDriver {

    public enum TYPE{
        DS180,
        DS270,
        DS360
    }

    private TYPE type;

    public CustomServoDriver(TYPE type){
        this.type = type;
    }

    public TYPE getType(){
        return type;
    }

    public static Stack<Double> generateCheckpoints(double startingPos, double targetPosition, int sections){
        Stack<Double> output = new Stack<>();
        double t = 1 / sections;
        output.push(startingPos);

        for (int i = 0; i < sections; i++) {
            output.push(startingPos + (targetPosition - startingPos) * t);
        }

        output.push(targetPosition);

        return EULArrays.stackFlip(output);
    }

    public static double followCheckpoints(double currentPos, Stack<Double> checkpoints){
        if (currentPos >= checkpoints.peek() && !checkpoints.empty()){checkpoints.pop();}
        return checkpoints.empty() ? currentPos : checkpoints.peek();
    }
}
