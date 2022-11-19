package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This routine runs a standard autonomous mode after reading YELLOW on the signal sleeve
 */

@Autonomous(name = "SpareAutoYellow", group = "actual")
public class SpareAutoYellow extends SpareAuto {

    public SpareAutoYellow() throws InterruptedException {
    }

    public void runOpMode() throws InterruptedException {

        robot.init( hardwareMap );

        waitForStart();

        runOpMode( Color.YELLOW, "Yellow" );
    }
}
