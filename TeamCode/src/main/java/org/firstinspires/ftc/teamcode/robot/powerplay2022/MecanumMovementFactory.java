package org.firstinspires.ftc.teamcode.robot.powerplay2022;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;

import java.util.HashMap;
import java.util.Objects;

public class MecanumMovementFactory {

    private HardwareMap hardwareMap;
    private BNO055IMU imu;

    private HashMap<String, DcMotor> motorMap;
    private Matrix4d mecanumPowerRatioMatrix;
    private double forwardBackMovt, strafeMovt, turnMovt;

    public MecanumMovementFactory(HardwareMap hardwareMap, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        this.hardwareMap = hardwareMap;
        initIMU(hardwareMap);

        motorMap = new HashMap<>();
        double normalizer = Math.max(Math.abs(forwardBackCoefficient) + Math.abs(strafingCoefficient) + Math.abs(turnCoefficient), 1);
        forwardBackMovt = forwardBackCoefficient / normalizer;
        strafeMovt = strafingCoefficient / normalizer;
        turnMovt = turnCoefficient / normalizer;
        initMatrix();
    }

    public void mapMotors(String frontLeft, boolean reverseFL, String backLeft, boolean reverseBL, String frontRight, boolean reverseFR, String backRight, boolean reverseBR){
        motorMap.clear();
        motorMap.put("FL", hardwareMap.get(DcMotor.class, frontLeft));
        motorMap.put("FR", hardwareMap.get(DcMotor.class, frontRight));
        motorMap.put("BL", hardwareMap.get(DcMotor.class, backLeft));
        motorMap.put("BR", hardwareMap.get(DcMotor.class, backRight));

        Objects.requireNonNull(motorMap.get("FL")).setDirection(reverseFL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("FR")).setDirection(reverseFR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("BL")).setDirection(reverseBL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("BR")).setDirection(reverseBR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

    }

    public void update(Vector3d control){
        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Vector3d weighedControl = new Vector3d(control.x * forwardBackMovt, control.y * strafeMovt, control.z * turnMovt);
        Vector4d in = (new Vector4d(weighedControl.x, weighedControl.y, weighedControl.z, 0)).div(Math.max(Math.abs(weighedControl.x) + Math.abs(weighedControl.y) + Math.abs(weighedControl.z), 1));
        Vector4d out = mecanumPowerRatioMatrix.times(in);

        //set the motor powers as referenced in the hashmap
        Objects.requireNonNull(motorMap.get("FL")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(out.y);
        Objects.requireNonNull(motorMap.get("FR")).setPower(out.z);
        Objects.requireNonNull(motorMap.get("BR")).setPower(out.w);
    }

    private void initIMU(HardwareMap hardwareMap){
        //set up IMU parameters for basic angle tracking
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;

        //pass those parameters to 'imu' when the hardware map fetches the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
    }

    private void initMatrix(){
        //4th column discards W component of the Vector4f multiplied with the matrix
        mecanumPowerRatioMatrix = new Matrix4d(new double[][]{
                {1,  1,  1, 0},
                {1, -1, -1, 0},
                {1, -1,  1, 0},
                {1,  1, -1, 0}
        });
    }
}
