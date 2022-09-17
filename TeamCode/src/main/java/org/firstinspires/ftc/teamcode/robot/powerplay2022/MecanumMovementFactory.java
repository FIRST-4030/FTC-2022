package org.firstinspires.ftc.teamcode.robot.powerplay2022;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.AccelIntegratorSemiImplicitEuler;

import java.util.HashMap;
import java.util.Objects;

public class MecanumMovementFactory {

    private HardwareMap hardwareMap;
    private BNO055IMU imu;

    private HashMap<String, DcMotor> motorMap;
    private Matrix4d mecanumPowerRatioMatrix;
    private double forwardBackMovt, strafeMovt, turnMovt;
    private Vector3d modulation = new Vector3d();
    private double acceptableError = 0.05;
    private Position recordedPos = new Position();
    public Vector4d out;

    public MecanumMovementFactory(HardwareMap hardwareMap, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        this.hardwareMap = hardwareMap;
        initIMU(hardwareMap);

        motorMap = new HashMap<>();
        //double normalizer = Math.max(Math.abs(forwardBackCoefficient) + Math.abs(strafingCoefficient) + Math.abs(turnCoefficient), 1);
        double normalizer = 1;
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

    public void update(Vector3d control, boolean fieldCentric){
        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Matrix3d rot = fieldCentric ? Matrix3d.makeAffineRotation(-angles.firstAngle) : new Matrix3d();
        Vector3d weighedControl = rot.times(new Vector3d(control.x * forwardBackMovt, control.y * strafeMovt, control.z * turnMovt));
        Vector4d in = (new Vector4d(weighedControl.x, weighedControl.y, weighedControl.z, 0));
        out = mecanumPowerRatioMatrix.times(in).div(Math.max(Math.abs(weighedControl.x) + Math.abs(weighedControl.y) + Math.abs(weighedControl.z), 1));

        //set the motor powers as referenced in the hashmap
        Objects.requireNonNull(motorMap.get("FL")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(out.y);
        Objects.requireNonNull(motorMap.get("FR")).setPower(out.z);
        Objects.requireNonNull(motorMap.get("BR")).setPower(out.w);
    }

    public void update(){
        update(modulation, true);
    }

    public boolean alignX(Vector2d input){

        Position currentPos = imu.getPosition();
        Velocity currentVelocity = imu.getVelocity();
        boolean outputBool = (input.x - currentPos.x) <= acceptableError;

        calcPos(input);

        Vector3d output = new Vector3d();
        output.x = modulation.x;
        output.y = 0;
        output.z = 0;
        /*
        if(!outputBool) {
            update(output, true);
        }

         */
        return outputBool;
    }

    public boolean alignY(Vector2d input){
        Position currentPos = imu.getPosition();
        Velocity currentVelocity = imu.getVelocity();
        boolean outputBool = (input.x - currentPos.x) <= acceptableError;

        calcPos(input);

        Vector3d output = new Vector3d();
        output.x = 0;
        output.y = modulation.y;
        output.z = 0;

        /*
        update(output, true);

         */
        return outputBool;
    }

    private void calcPos(Vector2d target){
        Vector2d delta = new Vector2d();
        Position integratedPos = imu.getPosition();

        integratedPos.x -= recordedPos.x;
        integratedPos.y -= recordedPos.y;
        integratedPos.z -= recordedPos.z;

        delta.x = target.x - integratedPos.x;
        delta.y = target.y - integratedPos.y;
        double distance =  delta.length();
        boolean far = distance > acceptableError * 2;

        modulation.x = EULMathEx.doubleClamp(-1, 1, far ? delta.x / distance : delta.x);
        modulation.y = EULMathEx.doubleClamp(-1, 1, far ? delta.y / distance : delta.y);

    }

    private void initIMU(HardwareMap hardwareMap){
        //set up IMU parameters for basic angle tracking
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.accelerationIntegrationAlgorithm = new AccelIntegratorSemiImplicitEuler();
        imuParams.loggingEnabled = false;

        //pass those parameters to 'imu' when the hardware map fetches the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);

        recordedPos = imu.getPosition();
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
