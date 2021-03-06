package org.firstinspires.ftc.teamcode.utils.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.general.Available;

public class ServoFTC implements Available {
    private Servo servo;
    private static final float ABS_MIN = 0.0f;
    private final static float ABS_MAX = 1.0f;
    private float min = ABS_MIN;
    private float max = ABS_MAX;

    public ServoFTC(HardwareMap map, Telemetry telemetry, ServoConfig config) {
        if (config == null) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Null config");
            return;
        }
        if (config.name == null || config.name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null/empty name");
        }
        try {
            servo = map.servo.get(config.name);
            if (config.reverse) {
                servo.setDirection(Servo.Direction.REVERSE);
            }
            this.min = config.min;
            this.max = config.max;
        } catch (Exception e) {
            servo = null;
            telemetry.log().add(this.getClass().getSimpleName() + "No such device: " + config.name);
        }
    }

    public boolean isAvailable() {
        return servo != null;
    }

    public void setPosition(float position) {
        if (position < min) {
            position = min;
        } else if (position > max) {
            position = max;
        }
        setPositionRaw(position);
    }

    public void setPositionRaw(float position) {
        if (!isAvailable()) {
            return;
        }
        if (position < ABS_MIN) {
            position = ABS_MIN;
        } else if (position > ABS_MAX) {
            position = ABS_MAX;
        }
        servo.setPosition(position);
    }

    public float getPosition() {
        if (!isAvailable()) {
            return ABS_MIN;
        }
        return (float) servo.getPosition();
    }

    public void min() {
        setPositionRaw(min);
    }

    public void max() {
        setPositionRaw(max);
    }

    public void toggle() {
        float mid = (max + min) / 2;
        if (getPosition() >= mid) {
            min();
        } else {
            max();
        }
    }

    public float getMin() {
        return min;
    }

    public float getMax() {
        return max;
    }
}
