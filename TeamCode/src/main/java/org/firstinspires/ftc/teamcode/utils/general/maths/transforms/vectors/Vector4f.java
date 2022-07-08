package org.firstinspires.ftc.teamcode.utils.general.maths.transforms.vectors;

import java.io.IOException;

public class Vector4f implements VectorImpl {
    public static final Vector4f instance = new Vector4f(0, 0, 0, 0);
    public float x, y, z, w;

    public Vector4f(){
        x = 0; y = 0; z = 0; w = 1;
    }

    public Vector4f(float[] list){
        if (list.length == 4){
            x = list[0]; y = list[1]; z = list[2]; w = list[3];
        } else {
            IOException e = new IOException("Array length is not 4!");
            e.printStackTrace();
        }
    }

    public Vector4f(float x, float y, float z){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = 1;
    }

    public Vector4f(float x, float y, float z, float w){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public float getX(){
        return this.x;
    }

    public float getY(){
        return this.y;
    }

    public float getZ(){
        return this.z;
    }

    public float getW(){
        return this.w;
    }

    public void setX(float nx){
        this.x = nx;
    }

    public void setY(float ny){
        this.y = ny;
    }

    public void setZ(float nz){
        this.z = nz;
    }

    public void setW(float nw){
        this.w = nw;
    }

    public float length(){
        return (float) Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
    }

    public void normalize(){
        float length = this.length();
        this.x /= length;
        this.y /= length;
        this.z /= length;
        this.w /= length;
    }

    public Vector4f normalized(){
        float length = this.length();
        float nx = this.x, ny = this.y,  nz = this.z,  nw = this.w;
        nx /= length;
        ny /= length;
        nz /= length;
        nw /= length;
        return new Vector4f(nx, ny, nz, nw);
    }


    public void add(Vector4f b){
        this.x += b.x;
        this.y += b.y;
        this.z += b.z;
        this.w += b.w;
    }

    public void sub(Vector4f b){
        this.x -= b.x;
        this.y -= b.y;
        this.z -= b.z;
        this.w -= b.w;
    }

    public void mul(float b){
        this.x *= b;
        this.y *= b;
        this.z *= b;
        this.w *= b;
    }

    public void div(float b){
        this.x /= b;
        this.y /= b;
        this.z /= b;
        this.w /= b;
    }

    public float dot(Vector4f b){
        return this.x * b.x + this.y * b.y + this.z * b.z;
    }

    public static Vector4f add(Vector4f a, Vector4f b){
        return new Vector4f(a.x + b.x ,a.y + b.y, a.z + b.z, a.w + b.w);
    }

    public static Vector4f sub(Vector4f a, Vector4f b){
        return new Vector4f(a.x - b.x ,a.y - b.y, a.z - b.z, a.w - b.w);
    }

    public static Vector4f mul(Vector4f a, float b){
        return new Vector4f(a.x * b, a.y * b, a.z * b, a.w * b);
    }

    public static Vector4f div(Vector4f a, float b){
        return new Vector4f(a.x / b, a.y / b, a.z / b, a.w / b);
    }

    public static float dot(Vector4f a, Vector4f b){
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    public Vector3f getAsVec3f(){
        return new Vector3f(this.x, this.y, this.z);
    }

    @Override
    public String toString(){ return ("X: " + this.x + " Y: " + this.y + " Z: " + this.z + " W: " + this.w);}

    @Override
    public float[] getAsList() {
        return new float[]{x, y, z, w};
    }
}
