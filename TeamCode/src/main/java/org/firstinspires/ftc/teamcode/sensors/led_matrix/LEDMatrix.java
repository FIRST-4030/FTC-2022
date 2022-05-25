package org.firstinspires.ftc.teamcode.sensors.led_matrix;

import org.firstinspires.ftc.teamcode.sensors.led_matrix.driver.BLINKING_MODE;
import org.firstinspires.ftc.teamcode.utils.Available;

public interface LEDMatrix extends Available {
    // other things
    void setDisplayBuffer(byte[] displayBuffer);
    void reset();

    // dragging this shit out of the driver class and into the interface
    void clear();
    void write();
    void setBrightness(int brightness);
    void setBlinking(BLINKING_MODE mode);
}
