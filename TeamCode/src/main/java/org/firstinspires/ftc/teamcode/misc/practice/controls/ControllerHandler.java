package org.firstinspires.ftc.teamcode.misc.practice.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.general.maths.transforms.vectors.Vector2f;

import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentHashMap;

public class ControllerHandler {

    private enum BUTTONS{
        A, B, Y, X,
        DPAD_DOWN, DPAD_RIGHT, DPAD_UP, DPAD_LEFT,
        LT, LB, RT, RB
    }

    private enum JOYSTICKS{
        RIGHT_X, RIGHT_Y,
        LEFT_X, LEFT_Y
    }

    private ConcurrentHashMap<String, Boolean> button_map = new ConcurrentHashMap<>();
    private Vector2f left_stick, right_stick;

    public ControllerHandler(Gamepad gamepad){
        Field[] gamepadFields = gamepad.getClass().getDeclaredFields();
        for (Field field: gamepadFields) {
            if (field.getType().equals(Boolean.class)){
                button_map.put(field.getName(), false);
            }
        }

        left_stick = new Vector2f();
        right_stick = new Vector2f();
    }
}
