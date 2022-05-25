package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Disabled
@Autonomous(name = "DistanceTest", group = "Test")
public class DistanceTest extends OpMode {
    // Hardware
    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;

    // Consts
    private static double SENSOR_TIME = 0.5;
    private static double leftSensorAccum = 0;
    private static double rightSensorAccum = 0;
    private static int objectPos = 0;

    // Members
    private ElapsedTime sensorTimer = new ElapsedTime();

    // Members
    private boolean done = false;
    private int autoStep = 0;
    private boolean leftinRange = false;
    private boolean rightinRange = false;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Distance Sensors
        try {
            distanceLeft = hardwareMap.get(DistanceSensor.class, "DL");
            distanceRight = hardwareMap.get(DistanceSensor.class, "DR");
        } catch (Exception e) {
            telemetry.log().add("Could not find distance sensors");
            error = true;
        }

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        done = false;
        autoStep = 0;
    }

    @Override
    public void loop() {
        // Stop when the autoSteps are complete
        if (done) {
            requestOpModeStop();
            return;
        }

        // Step through the auto commands
        switch (autoStep) {
            case 0:
                sensorTimer.reset();
                leftSensorAccum = 0;
                autoStep++;
                break;
            case 1:
                leftSensorAccum = (leftSensorAccum * 0.9) + (0.1 * distanceLeft.getDistance(DistanceUnit.INCH));
                if (sensorTimer.seconds() > SENSOR_TIME) {
                    autoStep++;
                }
                break;
            case 2:
                if (leftSensorAccum <= 19) {
                    leftinRange = true;
                }
                autoStep++;
                break;
            case 3:
                sensorTimer.reset();
                rightSensorAccum = 0;
                autoStep++;
                break;
            case 4:
                rightSensorAccum = (rightSensorAccum * 0.9) + (0.1 * distanceRight.getDistance(DistanceUnit.INCH));
                if (sensorTimer.seconds() > SENSOR_TIME) {
                    autoStep++;
                }
                break;
            case 5:
                if (rightSensorAccum <= 19) {
                    rightinRange = true;
                }
                autoStep++;
                break;
            case 6:
                if (leftinRange && !rightinRange) {
                    objectPos = 1;
                    telemetry.addData("Position: ", objectPos);
                } else if (!leftinRange && rightinRange){
                    objectPos = 3;
                    telemetry.addData("Position: ", objectPos);
                } else if (!leftinRange && !rightinRange) {
                    objectPos = 2;
                    telemetry.addData("Position: ", objectPos);
                } else {
                    objectPos = 0;
                    telemetry.addData("Position: ", objectPos);
                }
                break;
            // If we get past the end stop and end the loop
            default:
                done = true;
                break;
        }
    }

    @Override
    public void stop() {
    }
}