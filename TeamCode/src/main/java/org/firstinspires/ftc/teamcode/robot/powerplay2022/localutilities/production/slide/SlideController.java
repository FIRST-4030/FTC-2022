package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideController {

    public enum LEVEL{
        REST, LOW, MIDDLE, HIGH
    }

    private DcMotor left, right;

    //public int leftEncoderPosition = 0;
    public int rightEncoderPosition = 0;
    public int tickTolerance = 5;

    private int leftLastEncoderPosition = 0;
    private int rightLastEncoderPosition = 0;

    private boolean inUse = false;

    public SlideController(HardwareMap hardwareMap, String leftMotorName, boolean invertLeft, String rightMotorName, boolean invertRight){
        left = hardwareMap.dcMotor.get(leftMotorName);
        right = hardwareMap.dcMotor.get(rightMotorName);

        /*
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(0);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setDirection(invertLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(0);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setDirection(invertRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(double deltaTime, LEVEL level, double slidePower){

        switch (level){
            case REST:
                //left.setTargetPosition(0);
                right.setTargetPosition(0);
                break;
            case LOW:
                //left.setTargetPosition(540 / 3 - 50);
                right.setTargetPosition(540 / 3 - 50);
                break;
            case MIDDLE:
                //left.setTargetPosition(540 / 3 + 100);
                right.setTargetPosition(540 / 3 + 100);
                break;
            case HIGH:
                //left.setTargetPosition(540 / 3 + 250);
                right.setTargetPosition(540 / 3 + 250);
                break;
        }

        if (Math.abs(rightLastEncoderPosition - right.getTargetPosition()) >= tickTolerance) {
            //left.setPower(slidePower);
            right.setPower(slidePower);
        } else {
            right.setPower(0);
        }

        inUse = !(level == LEVEL.REST);

        //leftLastEncoderPosition = leftEncoderPosition;
        rightLastEncoderPosition = rightEncoderPosition;

    }

    public void logMotorPos(Telemetry telemetry){
        telemetry.addData("LSRM Encoder Position: ", right.getCurrentPosition());
        telemetry.addData("LSLM Encoder Position: ", left.getCurrentPosition());
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
