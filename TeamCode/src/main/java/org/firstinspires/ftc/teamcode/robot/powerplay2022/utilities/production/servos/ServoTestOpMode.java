package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "Arm Servo Testing", group = "Tester")
public class ServoTestOpMode extends LoopUtil {

    public static ServoFTC A;
    public static ServoConfig configA;

    public static InputHandler gamepadHandler;
    public static boolean enableJoystick;
    public static double commandedPosition;

    @Override
    public void opInit() {
        configA = new ServoConfig("A");
        A = new ServoFTC(hardwareMap, telemetry, configA);

        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        enableJoystick = false;
        commandedPosition = 0.0;
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        gamepadHandler.loop();
        if (gamepadHandler.up("D1:LT")){
            enableJoystick = !enableJoystick;
        }

        if (gamepadHandler.up("D1:DPAD_UP")){ //increase outputSpeed by decimalPlace
            commandedPosition += 0.01;
            commandedPosition = EULMathEx.doubleClamp(0, 1, commandedPosition);
        }
        if (gamepadHandler.up("D1:DPAD_DOWN")){ //decrease outputSpeed by decimalPlace
            commandedPosition -= 0.01;
            commandedPosition = EULMathEx.doubleClamp(0, 1, commandedPosition);
        }

        if (enableJoystick){
            A.setPosition(commandedPosition);
        }

        telemetry.addData("Commanded Position: ", commandedPosition);
        telemetry.addData("Servo Turn Position: ", A.getPosition());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
