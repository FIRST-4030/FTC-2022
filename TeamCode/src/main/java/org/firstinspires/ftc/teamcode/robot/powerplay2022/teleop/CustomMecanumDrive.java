package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.VerletIntegrator;

import java.util.HashMap;
import java.util.Objects;

public class CustomMecanumDrive {

    private HardwareMap hardwareMap;
    public BNO055IMU imu;
    public VerletIntegrator verletIntegrator;

    private HashMap<String, DcMotor> motorMap;
    private Matrix4d mecanumPowerRatioMatrix;
    private double forwardBackMovt, strafeMovt, turnMovt;
    private double coefficientSum;
    private Vector4d out;

    private double outputMultiplier = 1;

    public CustomMecanumDrive(HardwareMap hardwareMap, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        this.hardwareMap = hardwareMap;
        initIMU(hardwareMap);

        motorMap = new HashMap<>();
        forwardBackMovt = forwardBackCoefficient;
        strafeMovt = strafingCoefficient;
        turnMovt = turnCoefficient;

        coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);

        initMatrix();
        verletIntegrator = new VerletIntegrator();
        verletIntegrator.init();
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

        motorMap.get("FL").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("FL").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("FL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorMap.get("FR").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("FR").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("FR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorMap.get("BL").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("BL").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("BL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorMap.get("BR").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("BR").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("BR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Vector3d control, boolean fieldCentric, double dt){
        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Matrix3d rot = fieldCentric ? Matrix3d.makeAffineRotation(-angles.firstAngle) : new Matrix3d();
        Vector3d rotated = rot.times(control.unaryMinus());
        Vector4d internalControl = new Vector4d(rotated.x, rotated.y, rotated.z, 1);
        out = mecanumPowerRatioMatrix.times(internalControl).div(Math.max(coefficientSum, 1));

        //set the motor powers as referenced in the hashmap
        Objects.requireNonNull(motorMap.get("FL")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(out.y);
        Objects.requireNonNull(motorMap.get("FR")).setPower(out.z);
        Objects.requireNonNull(motorMap.get("BR")).setPower(out.w);

        verletIntegrator.integrate(imu.getLinearAcceleration(), dt);
    }

    private void initIMU(HardwareMap hardwareMap){
        //set up IMU parameters for basic angle tracking
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;

        //pass those parameters to 'imu' when the hardware map fetches the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
    }

    private void initMatrix(){
        //4th column discards W component of the Vector4f multiplied with the matrix
        mecanumPowerRatioMatrix = (new Matrix4d(new double[][]{
                {strafeMovt,  forwardBackMovt,  turnMovt, 0},
                {strafeMovt, -forwardBackMovt, -turnMovt, 0},
                {strafeMovt, -forwardBackMovt,  turnMovt, 0},
                {strafeMovt,  forwardBackMovt, -turnMovt, 0}
        }).times(1/coefficientSum).times(-outputMultiplier));
    }

    private void initCoefficients(){
        coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);
    }

    public double getOutputMultiplier(){
        return outputMultiplier;
    }

    public void setOutputMultiplier(double nPower){
        this.outputMultiplier = EULMathEx.doubleClamp(-1, 1, nPower);
        initMatrix();
    }
}
