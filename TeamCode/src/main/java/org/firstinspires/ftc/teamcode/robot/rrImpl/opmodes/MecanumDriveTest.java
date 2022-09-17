package org.firstinspires.ftc.teamcode.robot.rrImpl.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.MecanumMovementFactory;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

@Config
@Autonomous(name="RRIMPLMecanumTest", group="Test")
public class MecanumDriveTest extends LoopUtil {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    MecanumMovementFactory MFac;

    RevColorRange RCR1;
    //drive
    public static SampleMecanumDrive drive;

    //poses
    public static Pose2d startingPose = new Pose2d( 0, 0);

    @Override
    public void opInit() {
        //initialize drive and virtual position
        //drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(startingPose);

        motorFrontLeft = hardwareMap.get(DcMotor.class,"FL");
        motorBackLeft = hardwareMap.get(DcMotor.class,"BL");
        motorFrontRight = hardwareMap.get(DcMotor.class,"FR");
        motorBackRight = hardwareMap.get(DcMotor.class,"BR");
        double coefficientFactor = 1;
        MFac = new MecanumMovementFactory(hardwareMap, 0.8 * coefficientFactor, 0.6 * coefficientFactor, 0.4 * coefficientFactor);
        MFac.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);
        RCR1 = new RevColorRange(hardwareMap, telemetry, "CS");
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        //build trajectory that goes forward by 24 inches
        //Trajectory tra = drive.trajectoryBuilder(startingPose)
        // .forward(24)
        // .build();

        //tra.end();
        //submit trajectory
        //drive.followTrajectory(tra);
        /*
        double rx = (gamepad1.right_stick_x)/2;
        double y = (-gamepad1.left_stick_y)/2;
        double x = (gamepad1.left_stick_x)/2;

         */

        double x = gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        /*
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = ( y - x - rx) / denominator;
        double frontRightPower = ( y - x + rx) / denominator;
        double backRightPower = ( y + x - rx) / denominator;

        motorFrontLeft.setPower(-frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackRight.setPower(backRightPower);


         */
        MFac.update(new Vector3d(y, x, rx), true);
        if(!MFac.alignX(new Vector2d(0.5, 0.5)) && !MFac.alignY(new Vector2d(0.5, 0.5))){
            MFac.update();
        }
        NormalizedRGBA colorOutput = RCR1.color();
        telemetry.addData("Color Sensor output: \nR: " + colorOutput.red +
                "\nG: " + colorOutput.green +
                "\nB: " + colorOutput.blue +
                "\nA: " + colorOutput.alpha, "");

        telemetry.addData("Output vector: ", MFac.out.toString());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
