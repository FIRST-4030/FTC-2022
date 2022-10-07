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

    public enum SERVO{
        A,
        B,
        C
    }

    public static ServoFTC servoA, servoB, servoC;
    public static ServoConfig configA, configB, configC;

    public static InputHandler gamepadHandler;
    public static boolean enableJoystick;
    public static double commandedPosition;
    public static double commandedPositionMultiplier;
    public static SERVO servo = SERVO.A;

    @Override
    public void opInit() {
        configA = new ServoConfig("A",false, 0, 0.75);
        configB = new ServoConfig("B",false, 0, 1);
        configC = new ServoConfig("C",false, 0, 1);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);

        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        enableJoystick = false;
        commandedPosition = 0.0;
        commandedPositionMultiplier = 1;
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
            commandedPosition += commandedPositionMultiplier;
            commandedPosition = EULMathEx.doubleClamp(0, 1, commandedPosition);
        }
        if (gamepadHandler.up("D1:DPAD_DOWN")){ //decrease outputSpeed by decimalPlace
            commandedPosition -= commandedPositionMultiplier;
            commandedPosition = EULMathEx.doubleClamp(0, 1, commandedPosition);
        }

        if (gamepadHandler.up("D1:DPAD_LEFT") && !gamepad1.left_bumper){ //increase outputSpeed by decimalPlace
            commandedPositionMultiplier /= 10;
            commandedPositionMultiplier = EULMathEx.doubleClamp(0, 0.1, commandedPositionMultiplier);
        }
        if (gamepadHandler.up("D1:DPAD_RIGHT") && !gamepad1.left_bumper){ //decrease outputSpeed by decimalPlace
            commandedPositionMultiplier *= 10;
            commandedPositionMultiplier = EULMathEx.doubleClamp(0, 0.1, commandedPositionMultiplier);
        }

        if (gamepadHandler.up("D1:DPAD_LEFT") && gamepad1.left_bumper){
            switch (servo){
                case A: servo = SERVO.C; break;
                case B: servo = SERVO.A; break;
                case C: servo = SERVO.B; break;
            }
        }

        if (gamepadHandler.up("D1:DPAD_RIGHT") && gamepad1.left_bumper){
            switch (servo){
                case A: servo = SERVO.B; break;
                case B: servo = SERVO.C; break;
                case C: servo = SERVO.A; break;
            }
        }

        if (enableJoystick){
            switch (servo){
                case A: servoA.setPosition(commandedPosition); break;
                case B: servoB.setPosition(commandedPosition); break;
                case C: servoC.setPosition(commandedPosition); break;
            }
        }

        telemetry.addData("Commanded Multiplier: ", commandedPositionMultiplier);
        telemetry.addData("Commanded Position: ", commandedPosition);
        telemetry.addData("Servo: ", servo);
        telemetry.addData("Servo Turn Position: ", servoA.getPosition());
        telemetry.addData("Servo Turn Position: ", servoB.getPosition());
        telemetry.addData("Servo Turn Position: ", servoC.getPosition());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
