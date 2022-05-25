package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "DistanceTestDetailed", group = "Test")
public class DistanceTestDetailed extends LinearOpMode {

    private DistanceSensor Distance1;
    private DistanceSensor Distance2;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        Distance1 = hardwareMap.get(DistanceSensor.class, "DL");
        Distance2 = hardwareMap.get(DistanceSensor.class, "DR");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) Distance1;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("DL mm", String.format("%.01f mm", Distance1.getDistance(DistanceUnit.MM)));
            telemetry.addData("DL inch", String.format("%.01f in", Distance1.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DR mm", String.format("%.01f mm", Distance2.getDistance(DistanceUnit.MM)));
            telemetry.addData("DR inch", String.format("%.01f in", Distance2.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            /* telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur())); */

            telemetry.update();
        }
    }

}
