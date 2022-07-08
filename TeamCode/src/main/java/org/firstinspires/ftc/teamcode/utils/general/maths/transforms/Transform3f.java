package org.firstinspires.ftc.teamcode.utils.general.maths.transforms;

import org.firstinspires.ftc.teamcode.utils.general.maths.transforms.matrices.Matrix3f;
import org.firstinspires.ftc.teamcode.utils.general.maths.transforms.vectors.Vector3f;

public class Transform3f {

    private Matrix3f localRotation, localTranslation, localTransform;
    private Matrix3f globalRotation, globalTranslation, globalTransform;

    //default the matrices to identity matrices on construction of the object
    public Transform3f(){
        localRotation = new Matrix3f();
        localTranslation = new Matrix3f();
        localTransform = new Matrix3f();

        globalRotation = new Matrix3f();
        globalTranslation = new Matrix3f();
        globalTransform = new Matrix3f();
    }

    //transform vectors inputted into this

    public Vector3f transform(Vector3f in){
        Vector3f out = in;
        out = localTransform.matMul(out);
        out = globalTransform.matMul(out);
        return out;
    }

    //set the local matrices

    public void replaceLocalRotation(Matrix3f newRotation){
        Matrix3f lPos = this.localTranslation;
        Matrix3f lRot = newRotation;

        this.localTransform = Matrix3f.matMul(lPos, lRot);
    }

    public void replaceLocalTranslation(Matrix3f newTranslation){
        Matrix3f lPos = newTranslation;
        Matrix3f lRot = this.localRotation;

        this.localTransform = Matrix3f.matMul(lPos, lRot);
    }

    public void replaceLocalTransform(Matrix3f newTranslation, Matrix3f newRotation){
        Matrix3f lPos = newTranslation;
        Matrix3f lRot = newRotation;

        this.localTransform = Matrix3f.matMul(lPos, lRot);
    }


    //set the global matrices

    public void replaceGlobalRotation(Matrix3f newRotation){
        Matrix3f gPos = this.globalTranslation;
        Matrix3f gRot = newRotation;

        this.localTransform = Matrix3f.matMul(gPos, gRot);
    }

    public void replaceGlobalTranslation(Matrix3f newTranslation){
        Matrix3f gPos = newTranslation;
        Matrix3f gRot = this.globalRotation;

        this.localTransform = Matrix3f.matMul(gPos, gRot);
    }

    public void replaceGlobalTransform(Matrix3f newTranslation, Matrix3f newRotation){
        Matrix3f gPos = newTranslation;
        Matrix3f gRot = newRotation;

        this.localTransform = Matrix3f.matMul(gPos, gRot);
    }

    //get the local matrices
    public Matrix3f getLocalTranslation(){ return this.localTranslation; }
    public Matrix3f getLocalRotation() { return localRotation; }
    public Matrix3f getLocalTransform() { return localTransform; }

    //get the global matrices
    public Matrix3f getGlobalTranslation() { return globalTranslation; }
    public Matrix3f getGlobalRotation() { return globalRotation; }
    public Matrix3f getGlobalTransform() { return globalTransform; }
}
