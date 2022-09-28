package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.AnglePID;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.ColorView;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

@Config
@TeleOp(name = "PowerPlayMecanum", group = "Test")
public class DriveTest extends LoopUtil {

    public static CustomMecanumDrive drive  = null;
    public static Vector3d joystick = new Vector3d();
    public static AnglePID Apid = null;
    public static double[] angles = null;
    public static int angleIndex = 0;
    public static boolean lastStateLB = false, lastStateRB = false,currentStateLB = false, currentStateRB = false;
    public static double pGain = 0, iGain = 0, dGain = 0;


    //Algorithm-based correction (not PID)
    public static AlgorithmicCorrection correction;

    RevColorRange RCR2;
    ColorView CV2;

    @Override
    public void opInit() {

        Globals.opmode(this);
        Globals.input(this);

        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        drive.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);

        joystick = new Vector3d();

        pGain = 1/(Math.PI * 3);
        iGain = 0;
        dGain = 88;

        Apid = new AnglePID(1/Math.PI, 0.000001, 1/4000);
        Apid = new AnglePID(pGain, iGain, dGain);
        angles = new double[]{ 0, Math.PI/2,  Math.PI, -Math.PI/2};

        angleIndex = 0;

        lastStateRB = false;
        lastStateLB = false;
        currentStateLB = false;
        currentStateRB = false;

        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());

        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.SigmoidPiecewise());
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        lastStateRB = currentStateRB;
        lastStateLB = currentStateLB;

        currentStateLB =gamepad1.left_bumper;
        if(currentStateLB && !lastStateLB){
            angleIndex++;
        }

        currentStateRB = gamepad1.right_bumper;
        if(currentStateRB && !lastStateRB){
            angleIndex--;
        }

        if(angleIndex <= -1){
            angleIndex = 3;
        } else if (angleIndex >= 4){
            angleIndex = 0;
        }
        /*
        switch(angleIndex){
            case 0:
                Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                break;
            case 1:
                if(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle < -1*Math.PI/2 && drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle > -1*Math.PI){
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + 2*Math.PI);
                }else{
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                }
                break;
            case 2:
                if(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle < 0){
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + 2*Math.PI);
                }else{
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                }
                break;
            case 3:
                if(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle > Math.PI/2 && drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle < Math.PI){
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - 2*Math.PI);
                }else{
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                }
                break;
        }
         */



        correction.update(angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, false);

        joystick.x = gamepad1.left_stick_x;
        joystick.y = -gamepad1.left_stick_y;

        //joystick.z = gamepad1.right_stick_x; //manual steering

       // joystick.z = -1*Apid.correctionPower; //correction steering (PID)

        joystick.z = correction.getOutput() * 0.5; //an algorithm directly controls the rotation instead of adding to it



        CV2.update(RCR2.color(), RCR2.distance());

        drive.update(joystick, true, deltaTime);
        telemetry.addData("Angle: ", drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Acceleration: ", drive.getImu().getLinearAcceleration());
        AngularVelocity avel = drive.getImu().getAngularVelocity();
        telemetry.addData("Turn Velocity: ", new Vector3d(avel.xRotationRate, avel.yRotationRate, avel.zRotationRate));
        telemetry.addData("Angle Index: ", angleIndex);
        telemetry.addData("P: ", Apid.p);
        telemetry.addData("I: ", Apid.i);
        telemetry.addData("D: ", Apid.d);
        telemetry.addData("Correction: ", Apid.correctionPower);
        telemetry.addData("Color: ", CV2.getColor());
        telemetry.addData("Red: ", RCR2.color().red);
        telemetry.addData("Blue: ", RCR2.color().blue);
        telemetry.addData("Green: ", RCR2.color().green);
        telemetry.addData("Distance: ", RCR2.distance());

        correction.log(telemetry);
    }

    @Override
    public void opStop() {

    }
}
