package org.firstinspires.ftc.teamcode.robot.powerplay2022;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorView {
    public NormalizedRGBA colorInput;
    public double distance;

    public enum CMYcolors{
        CYAN,
        MAGENTA,
        YELLOW,
        NOT_DETECTED
    }

    public ColorView(NormalizedRGBA CI, double DIST){
        colorInput = CI;
        distance = DIST;
    }

    public CMYcolors getColor(){
        CMYcolors output = CMYcolors.NOT_DETECTED;
        if(distance < 1000000 && distance > 0) {
            if (colorInput.green < colorInput.red && colorInput.green < colorInput.blue) {
                output = CMYcolors.MAGENTA;
            } else if (colorInput.blue < colorInput.green && colorInput.blue < colorInput.red) {
                output = CMYcolors.YELLOW;
            } else {
                output = CMYcolors.CYAN;
            }
        }
        return output;
    }

    public void update(NormalizedRGBA CI, double DIST){
        colorInput = CI;
        distance = DIST;
    }
}
