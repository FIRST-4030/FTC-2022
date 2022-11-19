package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.utils.general.maths.misc.MathEx.pi;

/*
* This is a very crude attempt at showing how to use encoder ticks to move the robot
* in specified ways.
 */

@Autonomous(name = "DistanceDemo", group = "actual")
public class DistanceDemo extends LinearOpMode {

    enum Direction {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    }

    static final int CALIBRATION_TICKS = 1000;

    static final double POWER = 0.1;
    static final double STOP  = 0.0;
    static final double DISTANCE_TO_TRAVEL = 8.0;                 // inches

    static final double NEVEREST_ORBITAL_20_PPR = 537.6;
    static final double SPARE_CIRCUMFERENCE = pi * 4.0;           // inches

    static final int GOBILDA_PPR = 28;
    static final double GOBILDA_CIRCUMFERENCE = pi * 96 / 25.4;   // mm converted to inches

    static final double ticksPerInch = NEVEREST_ORBITAL_20_PPR;
    static final double circumference = SPARE_CIRCUMFERENCE;
//    static final int ticksPerInch = GOBILDA_PPR;
//    static final double circumference = GOBILDA_CIRCUMFERENCE;

    HardwareBot robot = new HardwareBot();

    public DistanceDemo() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        // calculate the ticks needed to travel
        double rotationsNeeded = DISTANCE_TO_TRAVEL / circumference;
        // calculate the number of ticks if the wheels were NOT mecanum
        int encoderDrivingTarget = (int)(rotationsNeeded*ticksPerInch);
//        telemetry.addData("Ticks (Calculated):", encoderDrivingTarget);
        encoderDrivingTarget = CALIBRATION_TICKS;
//        telemetry.addData("Ticks (Specified):", encoderDrivingTarget);

        goForAft( Direction.FORWARD, encoderDrivingTarget );
        moveTo( POWER );
        sleep(1000 );

        goForAft( Direction.REVERSE, encoderDrivingTarget );
        moveTo( POWER );

        goLeftRight( Direction.RIGHT, encoderDrivingTarget );
        moveTo( POWER );

        goLeftRight( Direction.LEFT, encoderDrivingTarget );
        moveTo( POWER );
    }

    public void goForAft( Direction _dir, int _distance ) {

        int absDistance = _distance;
        if (_dir == Direction.REVERSE) {
            absDistance = -absDistance;
        }

        robot.frontLeft.setTargetPosition(    absDistance  );
        robot.frontRight.setTargetPosition( -(absDistance) );
        robot.backLeft.setTargetPosition(   -(absDistance) );
        robot.backRight.setTargetPosition(    absDistance  );
    }

    public void goLeftRight( Direction _dir, int _distance ) {

        int absDistance = _distance;
        if (_dir == Direction.LEFT) {
            absDistance = -absDistance;
        }

        robot.frontLeft.setTargetPosition(  absDistance );
        robot.frontRight.setTargetPosition( absDistance );
        robot.backLeft.setTargetPosition(   absDistance );
        robot.backRight.setTargetPosition(  absDistance );
    }

    public void moveTo( double _power ) {

        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        setMode( DcMotor.RunMode.RUN_TO_POSITION );

        setPower( _power );

        while ( robot.backLeft.isBusy()  ||
                robot.backRight.isBusy() ||
                robot.frontLeft.isBusy() ||
                robot.frontRight.isBusy() ) {
            telemetry.addData("Back Left:", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left:", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", robot.frontRight.getCurrentPosition());
            telemetry.update();
        }
        setPower( 0.0 );
    }

    public void setMode( DcMotor.RunMode _mode ) {
        robot.backLeft.setMode(   _mode );
        robot.backRight.setMode(  _mode );
        robot.frontLeft.setMode(  _mode );
        robot.frontRight.setMode( _mode );
    }

    public void setPower( double _power ) {
        robot.backLeft.setPower(   _power );
        robot.backRight.setPower(  _power );
        robot.frontLeft.setPower(  _power );
        robot.frontRight.setPower( _power );
    }
}
