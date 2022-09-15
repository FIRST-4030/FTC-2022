package org.firstinspires.ftc.teamcode.robot.powerplay2022;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;

import java.util.HashMap;
import java.util.Objects;

public class MecanumMovementFactory {

    private BNO055IMU imu;

    private HashMap<String, DcMotor> motorMap;
    private Matrix4d coefficientMatrix;
    private double forwardBackMovt, strafeMovt, turnMovt;

    public MecanumMovementFactory(HardwareMap hardwareMap, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        motorMap = new HashMap<>();
        initIMU(hardwareMap);

        forwardBackMovt = forwardBackCoefficient;
        strafeMovt = strafingCoefficient;
        turnMovt = turnCoefficient;
    }

    public void mapMotors(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        motorMap.clear();
        motorMap.put("FL", frontLeft);
        motorMap.put("FR", frontLeft);
        motorMap.put("BL", frontLeft);
        motorMap.put("BR", frontLeft);
    }

    public void update(Vector3d control){
        Vector4d in = new Vector4d(control.x, control.y, control.z, 0).div(Math.max(Math.abs(control.x) + Math.abs(control.y) + Math.abs(control.z), 1));
        Vector4d out = coefficientMatrix.times(in);

        Objects.requireNonNull(motorMap.get("FL")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(out.y);
        Objects.requireNonNull(motorMap.get("FR")).setPower(out.z);
        Objects.requireNonNull(motorMap.get("BR")).setPower(out.w);
    }

    private void initIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
    }

    private void initMatrix(){
        coefficientMatrix = new Matrix4d(new double[][]{
                {strafeMovt, -forwardBackMovt, -turnMovt, 0},
                {strafeMovt,  forwardBackMovt,  turnMovt, 0},
                {strafeMovt,  forwardBackMovt, -turnMovt, 0},
                {strafeMovt, -forwardBackMovt,  turnMovt, 0}
        });
    }
}
