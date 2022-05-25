package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class TestColorSensor extends LinearOpMode {
    //Define variables for our color sensor
    ColorSensor color;
    DistanceSensor distance;
    double mm = 0;

    public boolean isAvailable() {
        return (color != null);
    }

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "C1");
        distance = hardwareMap.get(DistanceSensor.class, "C1");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values
        while (opModeIsActive()) {
            if (isAvailable()) {
                mm = distance.getDistance(DistanceUnit.MM);
            }
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            if (color.blue() <= color.red() * 0.8 && color.blue() <= color.green() *0.8) {
                telemetry.addData("Color Detected: ", "Yellow");
            } else {
                telemetry.addData("Color Detected: ", "Not Yellow");
            }
            telemetry.addData("Distance: ", mm);
            telemetry.update();
        }
    }
}
