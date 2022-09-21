package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.ImuIntegration;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.RK4Integrator;

import java.util.HashMap;
import java.util.Objects;

public class CustomMecanumDrive {

    protected HardwareMap hardwareMap;
    protected BNO055IMU imu;
    protected ImuIntegration integrator;

    protected HashMap<String, DcMotor> motorMap;
    protected Matrix4d mecanumPowerRatioMatrix;
    protected double forwardBackMovt, strafeMovt, turnMovt;
    protected double coefficientSum;
    protected Vector3d virtualJoystick;
    protected MecanumDriveTrajectory followTrajectory;
    protected boolean fieldCentricMode = true;
    protected Vector4d out;

    private double outputMultiplier = 1;

    public CustomMecanumDrive(HardwareMap hardwareMap, ImuIntegration integrator, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        this.hardwareMap = hardwareMap;
        this.integrator = integrator;
        initIMU(hardwareMap);

        this.motorMap = new HashMap<>();
        this.forwardBackMovt = forwardBackCoefficient;
        this.strafeMovt = strafingCoefficient;
        this.turnMovt = turnCoefficient;

        this.coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);

        initMatrix();
        integrator = new RK4Integrator();
        integrator.init();
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

        Acceleration acceleration = imu.getLinearAcceleration();
        Acceleration gravity = imu.getGravity();
        acceleration.xAccel -= gravity.xAccel;
        acceleration.yAccel -= gravity.yAccel;
        acceleration.zAccel -= gravity.zAccel;
        integrator.integrate(acceleration, dt);
    }

    public void followTrajectory(double dt){
        MecanumDriveState<?> state = followTrajectory.getCurrentStateStack().peek();
        if (state.isDone() && !followTrajectory.isCurrentStateStackEmpty()) {
            followTrajectory.pop();
            virtualJoystick.x = 0;
            virtualJoystick.y = 0;
            virtualJoystick.z = 0;
        }

        if(state.isDone()){} else {
            switch (state.condition.getName()){
                case "TimeCondition":
                    state.update(dt);
                    break;
                default:

            }
        }

        update(virtualJoystick, fieldCentricMode, dt);
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

    public BNO055IMU getImu(){
        return imu;
    }

    public ImuIntegration getIntegrator(){
        return integrator;
    }
}
