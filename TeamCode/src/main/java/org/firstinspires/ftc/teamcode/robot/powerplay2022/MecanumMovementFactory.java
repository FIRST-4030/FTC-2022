package org.firstinspires.ftc.teamcode.robot.powerplay2022;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.general.Available;

import java.util.HashMap;

public class MecanumMovementFactory implements Available {

    private BNO055IMU imu;

    private HashMap<String, DcMotor> motorMap;

    public MecanumMovementFactory(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        motorMap = new HashMap<>();
        mapMotors(frontLeft, frontRight, backLeft, backRight);

        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;



    }

    public void mapMotors(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        motorMap.clear();
        motorMap.put("FL", frontLeft);
        motorMap.put("FR", frontLeft);
        motorMap.put("BL", frontLeft);
        motorMap.put("BR", frontLeft);
    }

    @Override
    public boolean isAvailable() {
        return false;
    }
}
