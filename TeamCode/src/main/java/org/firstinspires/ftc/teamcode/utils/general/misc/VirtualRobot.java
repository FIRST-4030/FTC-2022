package org.firstinspires.ftc.teamcode.utils.general.misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.accelintegration.AccelIntegratorSemiImplicitEuler;

public class VirtualRobot {

    public enum Component{
        CURRENT_POSITION,
        CURRENT_VELOCITY,
        CURRENT_ACCELERATION,
        CURRENT_ANGULAR_VELOCITY,
        CURRENT_ORIENTATION,

        RECORDED_POSITION,
        RECORDED_ORIENTATION
    }

    private Telemetry telemetry;
    private BNO055IMU imu;

    private Position recordedPosition;
    private Orientation recordedOrientation;

    private Position currentPosition;
    private Velocity currentVelocity;
    private Acceleration currentAcceleration;
    private AngularVelocity currentAngularVelocity;
    private Orientation currentOrientation;

    public VirtualRobot(Telemetry telemetry, HardwareMap hardwareMap, String imuName){
        this.telemetry = telemetry;

        this.imu = hardwareMap.get(BNO055IMU.class, imuName);
    }

    public void init(){
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.accelerationIntegrationAlgorithm = new AccelIntegratorSemiImplicitEuler();
        imuParams.loggingEnabled = false;

        imu.initialize(imuParams);

        recordedPosition = imu.getPosition();
        recordedOrientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }

    public void update(){
        currentPosition = imu.getPosition();
        currentVelocity = imu.getVelocity();
        currentAcceleration = imu.getAcceleration();
    }

    public Vector3d getAsVector3d(Component field){
        Vector3d output = new Vector3d();

        switch (field){
            case CURRENT_POSITION:
                output.x = currentPosition.x;
                output.y = currentPosition.y;
                output.z = currentPosition.z;
                break;
            case CURRENT_VELOCITY:
                output.x = currentVelocity.xVeloc;
                output.y = currentVelocity.yVeloc;
                output.z = currentVelocity.zVeloc;
                break;
            case CURRENT_ACCELERATION:
                output.x = currentAcceleration.xAccel;
                output.y = currentAcceleration.yAccel;
                output.z = currentAcceleration.zAccel;
                break;
            case CURRENT_ANGULAR_VELOCITY:
                output.x = currentAngularVelocity.xRotationRate;
                output.y = currentAngularVelocity.yRotationRate;
                output.z = currentAngularVelocity.zRotationRate;
                break;
            case CURRENT_ORIENTATION:
                output.x = currentOrientation.firstAngle;
                output.y = currentOrientation.secondAngle;
                output.z = currentOrientation.thirdAngle;
                break;
            case RECORDED_POSITION:
                output.x = recordedPosition.x;
                output.y = recordedPosition.y;
                output.z = recordedPosition.z;
                break;
            case RECORDED_ORIENTATION:
                output.x = recordedOrientation.firstAngle;
                output.y = recordedOrientation.secondAngle;
                output.z = recordedOrientation.thirdAngle;
                break;
            default:
                output = null;
                break;
        }

        return output;
    }

    public Matrix4d getAsMatrix(boolean swapYZ){
        Vector3d rot = getAsVector3d(Component.CURRENT_ORIENTATION).minus(getAsVector3d(Component.RECORDED_ORIENTATION));
        Vector3d pos = getAsVector3d(Component.CURRENT_POSITION).minus(getAsVector3d(Component.RECORDED_POSITION));
        Matrix4d rotation = Matrix4d.makeAffineGenRotation(rot.x, swapYZ ? rot.z: rot.y, swapYZ ? rot.y : rot.z);
        return new Matrix4d(new double[][]{
                {rotation.matrix[0][0], rotation.matrix[0][1], rotation.matrix[0][2], pos.x},
                {rotation.matrix[1][0], rotation.matrix[1][1], rotation.matrix[1][2], pos.x},
                {rotation.matrix[2][0], rotation.matrix[2][1], rotation.matrix[2][2], pos.x},
                {0, 0, 0, 1}
        });
    }
}
