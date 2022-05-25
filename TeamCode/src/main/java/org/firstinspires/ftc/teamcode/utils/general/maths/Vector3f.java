package org.firstinspires.ftc.teamcode.utils.general.maths;

import java.io.IOException;

public class Vector3f {

    public float x, y, z;

    public Vector3f(){
        this.x = 0;
        this.y = 0;
        this.z = 1;
    }

    public Vector3f(float x, float y, float z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3f(float[] list){
        if (list.length == 3) {
            this.x = list[0];
            this.y = list[1];
            this.z = list[2];
        } else {
            IOException e = new IOException("Array length is not 3!");
            e.printStackTrace();
        }
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

    public void setX(float nx){ this.x = nx; }

    public void setY(float ny){ this.y = ny; }

    public void setZ(float nz){ this.z = nz; }

    public float length(){
        return (float) Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    public void normalize(){
        float l = this.length();
        this.x /= l;
        this.y /= l;
        this.z /= l;
    }

    public Vector3f normalized(){
        float l = this.length();
        float nx = this.x / l;
        float ny = this.y / l;
        float nz = this.z / l;
        return new Vector3f(nx, ny, nz);
    }

    public void add(Vector3f b){
        this.x += b.x;
        this.y += b.y;
        this.z += b.z;
    }

    public void sub(Vector3f b){
        this.x -= b.x;
        this.y -= b.y;
        this.z -= b.z;
    }

    public void mul(float b){
        this.x *= b;
        this.y *= b;
        this.z *= b;
    }

    public void div(float b){
        this.x /= b;
        this.y /= b;
        this.z /= b;
    }

    public float dot(Vector3f b){
        return this.x * b.x + this.y * b.y + this.z * b.z;
    }

    public static Vector3f add(Vector3f a, Vector3f b){
        return new Vector3f(a.x + b.x ,a.y + b.y, a.z + b.z);
    }

    public static Vector3f sub(Vector3f a, Vector3f b){
        return new Vector3f(a.x - b.x ,a.y - b.y, a.z - b.z);
    }

    public static Vector3f mul(Vector3f a, float b){
        return new Vector3f(a.x * b, a.y * b, a.z * b);
    }

    public static Vector3f div(Vector3f a, float b){
        return new Vector3f(a.x / b, a.y / b, a.z / b);
    }

    public static float dot(Vector3f a, Vector3f b){
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    @Override
    public String toString(){
        return ("X: " + this.x + " Y: " + this.y + " Z: " + this.z);
    }
}