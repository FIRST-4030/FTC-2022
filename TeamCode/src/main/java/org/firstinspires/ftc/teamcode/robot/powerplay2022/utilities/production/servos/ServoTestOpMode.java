package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "Arm Servo Testing", group = "Tester")
public class ServoTestOpMode extends LoopUtil {

    public static ServoFTC A;
    public static ServoConfig configA;

    @Override
    public void opInit() {
        configA = new ServoConfig("A");
        A = new ServoFTC(hardwareMap, telemetry, configA);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        telemetry.addData("Servo Turn Position: ", A.getPosition());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
