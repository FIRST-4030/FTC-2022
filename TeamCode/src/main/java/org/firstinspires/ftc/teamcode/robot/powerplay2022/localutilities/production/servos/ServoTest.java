package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

import java.util.Arrays;

@Config
@TeleOp(name = "Multi Servo Tester", group = "Tester")
public class ServoTest extends LoopUtil {
    public static ServoFTC[] servos;
    public static ServoConfig[] configs;
    public static double[] servoValues;

    public static int servoSelector = 0;
    public static InputHandler inputHandler;
    public static Vector2d joyStick = new Vector2d();

    @Override
    public void opInit() {

        configs = new ServoConfig[]{
                new ServoConfig("A",false, 0.0001, 0.83),
                new ServoConfig("B",true, 0.0001, 0.9999),
                new ServoConfig("C",true, 0.0001, 0.9999),
                new ServoConfig("D",true, 0.0001, 0.9999),
                new ServoConfig("R",true, 0.0001, 0.9999)
        };

        servos = new ServoFTC[]{
                new ServoFTC(hardwareMap, telemetry, configs[0]),
                new ServoFTC(hardwareMap, telemetry, configs[1]),
                new ServoFTC(hardwareMap, telemetry, configs[2]),
                new ServoFTC(hardwareMap, telemetry, configs[3]),
                new ServoFTC(hardwareMap, telemetry, configs[4])
        };

        servoValues = new double[servos.length];
        Arrays.fill(servoValues, 0d);

        joyStick = new Vector2d(0, 1);
        inputHandler = InputAutoMapper.normal.autoMap(this);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        handleInput(deltaTime * EULConstants.MS2SEC);

        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(servoValues[i]);
        }
    }

    public void handleInput(double deltaTime){
        inputHandler.loop();

        if (inputHandler.up("D1:DPAD_RIGHT")){
            servoSelector++;
        }

        if (inputHandler.up("D1:DPAD_LEFT")){
            servoSelector--;
        }

        if (inputHandler.held("D1:DPAD_UP")){
            servoValues[servoSelector] += 1 * deltaTime;
        }

        if (inputHandler.held("D1:DPAD_DOWN")){
            servoValues[servoSelector] -= 1 * deltaTime;
        }

        servoSelector = Math.floorMod(servoSelector, servos.length);
    }

    public void handleTelemetry(){
        telemetry.addData("CURRENT CONTROLLED SERVO: ", configs[servoSelector].name);
    }

    @Override
    public void opStop() {

    }
}
