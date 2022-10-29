package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.customDriver.CustomServoDriver;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

import java.util.Stack;

@Config
@TeleOp(name = "Servo Custom Driver Tester", group = "Tester")
public class ServoDriverTester extends LoopUtil {
    public static ServoFTC servo;
    public static ServoConfig config;
    public static Stack<Double> servoPath;
    public static SlideController slideController;

    @Override
    public void opInit() {
        config = new ServoConfig("R", false, 0, 1);

        servo = new ServoFTC(hardwareMap, telemetry, config);
        slideController = new SlideController(hardwareMap, "LSLM", true, "LSRM", false);
        servoPath = CustomServoDriver.SERVO360.generateServoPath(0, Math.PI * (181/180), CustomServoDriver.METHOD.LERP);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        slideController.update(deltaTime, SlideController.LEVEL.HIGH, 1);
        if (!servoPath.empty()){
            servo.setPosition(CustomServoDriver.followServoPath(servo.getPosition(), servoPath));
        }
        handleTelemetry();
    }

    public void handleTelemetry(){
        telemetry.addData("Servo Position: ", servo.getPosition());
        telemetry.addData("Checkpoint Position: ", servoPath.empty() ? servo.getPosition() : servoPath.peek());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {
        slideController.update(0, SlideController.LEVEL.LOW, 1);
    }
}
