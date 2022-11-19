package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This routine runs a standard autonomous mode after reading CYAN on the signal sleeve
 */

@Autonomous(name = "SpareAutoCyan", group = "actual")
public class SpareAutoCyan extends SpareAuto {

    public SpareAutoCyan() throws InterruptedException {
    }

    public void runOpMode() throws InterruptedException {

        robot.init( hardwareMap );

        waitForStart();

        runOpMode( Color.CYAN, "Cyan" );
    }
}
