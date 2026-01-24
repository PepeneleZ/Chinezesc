package org.firstinspires.ftc.teamcode.lib;

@SuppressWarnings("unused")
public class Feedforward {
    public double kV = 0, kA = 0, kStatic = 0, kP=0;
    public double dt=0, profiledVelocity=0, profiledAcceleration=0, currentVelocity=0, targetVelocity=0, maxAcceleration=0, startTime=0, duration=0, lastPosition=0, increasing=1, acceleration=0,lastVelocity=0;
    public long lastTime=0;

    public Feedforward(double kP,double kV, double kA, double kStatic ){
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }
    public Feedforward(double kP,double kV, double kA, double kStatic, double maxAcceleration ){
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.maxAcceleration = maxAcceleration;
    }

    public void setTarget(double target){
        reset();
        duration = (target-targetVelocity)/maxAcceleration;
        startTime = System.nanoTime();
        targetVelocity = target;
        if (target>targetVelocity) increasing = 1;
        else increasing = -1;
    }
    public void reset(){
        dt=0; profiledVelocity=0; profiledAcceleration=0; currentVelocity=0; lastTime=0; startTime=0; duration=0; lastPosition=0; acceleration=0; lastVelocity=0;
    }

    public double update(double currentPosition){
        long currentTime = System.nanoTime();
        double error;
        dt = (currentTime-lastTime)*1e-9;
        currentVelocity = (currentPosition-lastPosition)/dt;
        acceleration = (lastVelocity-currentVelocity)/dt;


        if ((profiledVelocity<currentVelocity&&increasing==1)||(profiledVelocity>currentVelocity&&increasing==-1))
            profiledVelocity += dt * maxAcceleration * increasing;
        else profiledVelocity = targetVelocity;

        error = profiledVelocity-currentVelocity; // for kP

        if ((currentTime-startTime)*1e-9<duration){
            profiledAcceleration = 0;
        }
        else {
            profiledAcceleration = maxAcceleration*increasing;
        }
        lastTime = currentTime;
        lastVelocity = currentVelocity;
        lastPosition = currentPosition;
        return error*kP + targetVelocity * kV + profiledAcceleration * kA + kStatic;
    }




}
