package org.firstinspires.ftc.teamcode.robot.rrImpl.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.ColorView;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.depreciated.movement.MecanumMovementFactory;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.depreciated.movement.MecanumTrajectory;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

import java.util.Stack;

@Config
@Autonomous(name="RRIMPLMecanumTest", group="Test")
public class MecanumDriveTest extends LoopUtil {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    MecanumMovementFactory MFac;
    RevColorRange RCR1;
    ColorView CV1;
    MecanumTrajectory[] cmdStacks;
    Stack<Pair<String, Double>> inUse;

    //drive
    public static SampleMecanumDrive drive;

    //poses
    public static Pose2d startingPose = new Pose2d( 0, 0);

    @Override
    public void opInit() {
        //initialize drive and virtual position
        //drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(startingPose);
        cmdStacks = new MecanumTrajectory[3];


        motorFrontLeft = hardwareMap.get(DcMotor.class,"FL");
        motorBackLeft = hardwareMap.get(DcMotor.class,"BL");
        motorFrontRight = hardwareMap.get(DcMotor.class,"FR");
        motorBackRight = hardwareMap.get(DcMotor.class,"BR");
        double coefficientFactor = 1;
        MFac = new MecanumMovementFactory(hardwareMap, 1 * coefficientFactor, 1.1 * coefficientFactor, 1 * coefficientFactor);
        MFac.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);
        RCR1 = new RevColorRange(hardwareMap, telemetry, "CS");
        CV1 = new ColorView(RCR1.color(), RCR1.distance());

        for(int i = 0; i < cmdStacks.length; i++){
            cmdStacks[i] = new MecanumTrajectory();
        }

        //cmdStack[0]
        cmdStacks[0].forward(-1);
        cmdStacks[0].idle(500);
        cmdStacks[0].turnLeft(1050);
        cmdStacks[0].idle(500);
        cmdStacks[0].right(800);
        cmdStacks[0].idle(-1);
        cmdStacks[0].build();

        inUse = cmdStacks[0].getCmdStack();

        MFac.cmdStack = inUse;
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


        double x = gamepad1.left_stick_y / 2;
        double y = -gamepad1.left_stick_x / 2;
        double rx = -gamepad1.right_stick_x / 2;
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


        //MFac.update(new Vector3d(y, x, rx), true);
        //if(!MFac.alignX(new Vector2d(0.5, 0.5)) && !MFac.alignY(new Vector2d(0.5, 0.5))){
        //    MFac.update();
        //}

        MFac.update(new Vector3d(y, x, rx), true);
        //MFac.execute(deltaTime);

        NormalizedRGBA colorOutput = RCR1.color();
        //CV1.update(RCR1.color(), RCR1.distance());
        telemetry.addData("Color Sensor output: \nR: " + colorOutput.red +
                "\nG: " + colorOutput.green +
                "\nB: " + colorOutput.blue +
                "\nA: " + colorOutput.alpha, "");

        telemetry.addData("Output vector: ", MFac.out.toString());
        telemetry.addData("Current X: ", MFac.getPos().x);
        telemetry.addData("Current Y: ", MFac.getPos().y);
        telemetry.addData("Distance: ", RCR1.distance());
        telemetry.addData("Color: ", CV1.getColor());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {
        MFac.dispose();
    }
}
