package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideController {

    private DcMotor left, right;

    private int leftEncoderPosition = 0;
    private int rightEncoderPosition = 0;

    private int leftLastEncoderPosition = 0;
    private int rightLastEncoderPosition = 0;

    private boolean inUse = false;

    public SlideController(HardwareMap hardwareMap, String leftMotorName, boolean invertLeft, String rightMotorName, boolean invertRight){
        left = hardwareMap.dcMotor.get(leftMotorName);
        right = hardwareMap.dcMotor.get(rightMotorName);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(invertLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setDirection(invertRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void update(double deltaTime, double scalar){
        left.setPower(scalar);
        right.setPower(scalar);

        inUse = (leftEncoderPosition == leftLastEncoderPosition) && (rightEncoderPosition == rightLastEncoderPosition);

        leftLastEncoderPosition = leftEncoderPosition;
        rightLastEncoderPosition = rightEncoderPosition;

    }

    public boolean isInUse(){
        return inUse;
    }

    public DcMotor getLeft(){
        return left;
    }

    public DcMotor getRight(){
        return right;
    }
}
