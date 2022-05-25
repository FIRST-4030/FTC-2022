package org.firstinspires.ftc.teamcode.sensors.led_matrix;

public class LEDMatrixConfig {
    public final String name;
    public final LED_MATRIX_TYPES type;

    public LEDMatrixConfig(LED_MATRIX_TYPES type, String name) {
        this.name = name;
        this.type = type;
    }
}
