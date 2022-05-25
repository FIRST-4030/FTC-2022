package org.firstinspires.ftc.teamcode.tfodohm.ODMain;

import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Plane3f;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.TFMathExtension;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Vector3f;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Vector4f;

public class CameraLens {
    //For the isBusy() method
    private boolean busy = false;

    //Camera Attributes
    private double hFOV = 1, vFOV = 1; //initialize with default values in just in case
    private final double zFar = 1000; //set the far plane/ will stretch the Z coordinate
    private final Plane3f xzPlane = Plane3f.XZ_PLANE; //define the plane for clipping the Camera Position to Calculated Vector 

    //NDC Img Attributes
    private Vector4f imgRight = new Vector4f(1, 0, 0, 1);
    private Vector4f imgBottom = new Vector4f(0, 1, 0, 1);
    private Vector4f imgCenter = new Vector4f(0, 0, 1, 1);

    //Describing transforms, most importantly, the rotation and translation
    private Matrix4f rotation = new Matrix4f();
    private Vector3f translation = new Vector3f();

    //Final output Matrix4f
    private Matrix4f imgToLocal = new Matrix4f();

    //Preset FOV value for the listed cameras (only the C270)
    public static final double[] C270_FOV = TFMathExtension.findFOV(3.58, 2.02, 4.11);

    public CameraLens(double[] fov){
        this.hFOV = fov[0];
        this.vFOV = fov[1];
    }

    public CameraLens(double[] fov, Vector3f lensPos, Matrix4f lensRot){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.translation = lensPos;
        this.rotation = lensRot;
    }

    public void setupCameraMatrix(){
        busy = true;

        //find half of the fov or aov
        double hHFOV = this.hFOV * 0.5;
        double hVFOV = this.vFOV * 0.5;

        //find the x and y scale for the camera fov
        double xScale = Math.tan(hHFOV) * zFar;
        double yScale = Math.tan(hVFOV) * zFar;

        //set the appropriate vectors with proper (X, Y, Z) scale
        this.imgRight.setX((float) xScale);
        this.imgBottom.setY((float) -yScale);
        this.imgCenter.setZ((float) this.zFar);

        //rotate those previous vectors
        this.imgRight = this.rotation.matMul(this.imgRight);
        this.imgBottom = this.rotation.matMul(this.imgBottom);
        this.imgCenter = this.rotation.matMul(this.imgCenter);

        //using a property of Matrices (columns bending the x, y, z... space)
        this.imgToLocal = new Matrix4f(
                new float[]{this.imgRight.getX(), this.imgBottom.getX(), this.imgCenter.getX(), this.translation.getX(),
                            this.imgRight.getY(), this.imgBottom.getY(), this.imgCenter.getY(), this.translation.getY(),
                            this.imgRight.getZ(), this.imgBottom.getZ(), this.imgCenter.getZ(), this.translation.getZ(),
                                               0,                     0,                     0,                       1}
        );

        busy = false;
    }

    public Vector3f findImgToLocal(Vector2f imgCoordinate){
        busy = true;
        Vector3f preClippedVector = imgToLocal.matMul(new Vector4f(imgCoordinate.getX(), imgCoordinate.getY(), 1, 1)).getAsVec3f();
        Vector3f output = this.xzPlane.getVector3fInt(this.translation, preClippedVector);
        busy = false;
        return output;
    }

    public Vector3f findImgToLocal(float imgX, float imgY){
        busy = true;
        Vector3f preClippedVector = imgToLocal.matMul(new Vector4f(imgX, imgY, 1, 1)).getAsVec3f();
        Vector3f output = this.xzPlane.getVector3fInt(this.translation, preClippedVector);
        busy = false;
        return output;
    }

    public void setTranslation(Vector3f newPosition){
        this.translation = newPosition;
        this.setupCameraMatrix();
    }

    public void setRotation(Matrix4f newRotation){
        this.rotation = newRotation;
        this.setupCameraMatrix();
    }

    public double getZFar(){
        return this.zFar;
    }

    public double getHFOV(){
        return this.hFOV;
    }

    public double getVFOV(){
        return this.vFOV;
    }

    public Vector3f getTranslation(){
        return this.translation;
    }

    public Vector4f getImgRight(){
        return this.imgRight;
    }

    public Vector4f getImgBottom(){
        return this.imgBottom;
    }

    public Vector4f getImgCenter(){
        return this.imgCenter;
    }

    public Plane3f getXZPlane(){
        return this.xzPlane;
    }

    public Matrix4f getRotation(){
        return this.rotation;
    }

    public Matrix4f getImgToLocal(){
        return this.imgToLocal;
    }

    @Override
    public String toString(){
        return  "CameraLens Attributes: " +
                "\nHFov: " + this.hFOV +
                "\nVFov: " + this.vFOV +
                "\nImgRight: " + this.imgRight +
                "\nImgBottom: " + this.imgBottom +
                "\nImgCenter: " + this.imgCenter +
                "\nTranslation: " + this.translation.toString() +
                "\nRotation: \n" + this.rotation.toString() +
                "\nImgToLocalMatrix: \n" + this.imgToLocal.toString();
    }

    public boolean isBusy() {
        return busy;
    }

    public static class CameraLensAlt{

        public enum LERP_SCHEMA{
            TC_TL_C_L,
            C_L_BC_BL,
            TC_TR_C_R,
            C_R_BC_BR
        }

        private Vector3f TL, TC, TR,
                          L,  C,  R,
                         BL, BC, BR;

        private Vector3f topX1, topX2, botX1, botX2;

        private LERP_SCHEMA schema;

        public CameraLensAlt(){
            initVector2fs();
        }

        public Vector3f calcImgToLocal(float imgX, float imgY){
            Vector3f output = new Vector3f();
            Vector3f topY = new Vector3f();
            Vector3f botY = new Vector3f();

            switch (this.schema){
                case TC_TL_C_L:
                case C_L_BC_BL:
                    topY = TFMathExtension.lerp(topX1, topX2, -imgX);
                    botY = TFMathExtension.lerp(botX1, botX2, -imgX);
                    output = TFMathExtension.lerp(botY, topY, -imgY);
                    break;
                case TC_TR_C_R:
                case C_R_BC_BR:
                    topY = TFMathExtension.lerp(topX1, topX2, imgX);
                    botY = TFMathExtension.lerp(botX1, botX2, imgX);
                    output = TFMathExtension.lerp(topY, botY, imgY);
                    break;
            }

            return output;
        }

        private void initVector2fs(){
            //init top Vec2fs
            TL = new Vector3f();
            TC = new Vector3f();
            TR = new Vector3f();

            //init mid Vec2fs
            L = new Vector3f();
            C = new Vector3f();
            R = new Vector3f();

            //init bot Vec2fs
            BL = new Vector3f();
            BC = new Vector3f();
            BR = new Vector3f();

            //init lerp vectors
            topX1 = new Vector3f();
            topX2 = new Vector3f();
            botX1 = new Vector3f();
            botX2 = new Vector3f();
        }

        public void setSchema(LERP_SCHEMA newSchema){
            this.schema = newSchema;
            updateVectors();
        }

        public void updateVectors(){
            switch (this.schema){
                case C_R_BC_BR:
                    topX1 = C;
                    topX2 = R;
                    botX1 = BC;
                    botX2 = BR;
                    break;

                case TC_TR_C_R:
                    topX1 = TC;
                    topX2 = TR;
                    botX1 = C;
                    botX2 = R;
                    break;

                case C_L_BC_BL:
                    topX1 = C;
                    topX2 = L;
                    botX1 = BC;
                    botX2 = BL;
                    break;
                case TC_TL_C_L:
                    topX1 = TC;
                    topX2 = TL;
                    botX1 = C;
                    botX2 = L;
                    break;
            }
        }

        public Vector3f getTL() {
            return TL;
        }

        public void setTL(Vector3f TL) {
            this.TL = TL;
            updateVectors();
        }

        public Vector3f getTC() {
            return TC;
        }

        public void setTC(Vector3f TC) {
            this.TC = TC;
            updateVectors();
        }

        public Vector3f getTR() {
            return TR;
        }

        public void setTR(Vector3f TR) {
            this.TR = TR;
            updateVectors();
        }

        public Vector3f getL() {
            return L;
        }

        public void setL(Vector3f l) {
            L = l;
            updateVectors();
        }

        public Vector3f getC() {
            return C;
        }

        public void setC(Vector3f c) {
            C = c;
            updateVectors();
        }

        public Vector3f getR() {
            return R;
        }

        public void setR(Vector3f r) {
            R = r;
            updateVectors();
        }

        public Vector3f getBL() {
            return BL;
        }

        public void setBL(Vector3f BL) {
            this.BL = BL;
            updateVectors();
        }

        public Vector3f getBC() {
            return BC;
        }

        public void setBC(Vector3f BC) {
            this.BC = BC;
            updateVectors();
        }

        public Vector3f getBR() {
            return BR;
        }

        public void setBR(Vector3f BR) {
            this.BR = BR;
            updateVectors();
        }
    }
}
