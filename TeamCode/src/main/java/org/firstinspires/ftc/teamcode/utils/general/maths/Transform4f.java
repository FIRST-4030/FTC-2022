package org.firstinspires.ftc.teamcode.utils.general.maths;

public class Transform4f {

    private Matrix4f localRotation, localTranslation, localTransform;
    private Matrix4f globalRotation, globalTranslation, globalTransform;

    //default the matrices to identity matrices on construction of the object
    public Transform4f(){
        localRotation = new Matrix4f();
        localTranslation = new Matrix4f();
        localTransform = new Matrix4f();

        globalRotation = new Matrix4f();
        globalTranslation = new Matrix4f();
        globalTransform = new Matrix4f();
    }

    //transform vectors inputted into this

    public Vector4f transform(Vector4f in){
        Vector4f out = in;
        out = localTransform.matMul(out);
        out = globalTransform.matMul(out);
        return out;
    }

    //set the local matrices

    public void replaceLocalRotation(Matrix4f newRotation){
        Matrix4f lPos = this.localTranslation;
        Matrix4f lRot = newRotation;

        this.localTransform = Matrix4f.matMul(lPos, lRot);
    }

    public void replaceLocalTranslation(Matrix4f newTranslation){
        Matrix4f lPos = newTranslation;
        Matrix4f lRot = this.localRotation;

        this.localTransform = Matrix4f.matMul(lPos, lRot);
    }

    public void replaceLocalTransform(Matrix4f newTranslation, Matrix4f newRotation){
        Matrix4f lPos = newTranslation;
        Matrix4f lRot = newRotation;

        this.localTransform = Matrix4f.matMul(lPos, lRot);
    }


    //set the global matrices

    public void replaceGlobalRotation(Matrix4f newRotation){
        Matrix4f gPos = this.globalTranslation;
        Matrix4f gRot = newRotation;

        this.localTransform = Matrix4f.matMul(gPos, gRot);
    }

    public void replaceGlobalTranslation(Matrix4f newTranslation){
        Matrix4f gPos = newTranslation;
        Matrix4f gRot = this.globalRotation;

        this.localTransform = Matrix4f.matMul(gPos, gRot);
    }

    public void replaceGlobalTransform(Matrix4f newTranslation, Matrix4f newRotation){
        Matrix4f gPos = newTranslation;
        Matrix4f gRot = newRotation;

        this.localTransform = Matrix4f.matMul(gPos, gRot);
    }

    //get the local matrices
    public Matrix4f getLocalTranslation(){ return this.localTranslation; }
    public Matrix4f getLocalRotation() { return localRotation; }
    public Matrix4f getLocalTransform() { return localTransform; }

    //get the global matrices
    public Matrix4f getGlobalTranslation() { return globalTranslation; }
    public Matrix4f getGlobalRotation() { return globalRotation; }
    public Matrix4f getGlobalTransform() { return globalTransform; }
}
