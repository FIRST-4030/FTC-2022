package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.slide.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "PowerPlayLinearSlideTest", group = "Testers")
public class PowerPlayLinearSlideTest extends LoopUtil {

    public InputHandler inputHandler;
    public SlideController controller;

    public DcMotor left, right;

    public double linearSlideSpeed = 0.5;
    public double lsInput = 0;

    @Override
    public void opInit() {
        inputHandler = InputAutoMapper.normal.autoMap(this);
        controller = new SlideController(hardwareMap, "LSLM", true, "LSRM", false);

        left = controller.getLeft();
        right = controller.getRight();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        handleInput(deltaTime);
        controller.update(deltaTime, lsInput);
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }

    public void handleInput(double deltaTime){
        if (inputHandler.held("D1:DPAD_UP")){
            lsInput = linearSlideSpeed * (deltaTime * EULConstants.MS2SEC);
        }

        if (inputHandler.held("D1:DPAD_DOWN")){
            lsInput = -linearSlideSpeed * (deltaTime * EULConstants.MS2SEC);
        }
    }

    public void outputTelemetry(){
        telemetry.addData("Left Encoder Ticks: ", left.getCurrentPosition());
        telemetry.addData("Right Encoder Ticks: ", right.getCurrentPosition());
    }
}
