package org.firstinspires.ftc.teamcode.lib;

import androidx.core.util.Supplier;

@SuppressWarnings("unused")
public class PIDF extends PIDF.PID {
    Supplier<Double> kFun = () -> 0d;

    public PIDF(double kP, double kI, double kD, Supplier<Double> kFun) {
        super(kP, kI, kD);
        this.kFun = kFun;
    }

    @Override
    public double update(int currentTicks){
        int e = targetPos - currentTicks;
        int delta_e = lastError - e;
        sumError += e;
        lastError = e;
        return (e * kP + delta_e * kD + sumError * kI + kFun.get());
    }

    @SuppressWarnings("unused")
    public static class PID {
        public double kP, kI, kD;
        protected int targetPos = 0, lastError = 0, sumError = 0;

        public PID(double kP, double kI, double kD){
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public void resetIntegral(){
            sumError = 0;
        }

        public void setTargetPosition(int target){
            targetPos = target;
        }

        public double update(int currentTicks){
            int e = targetPos - currentTicks;
            int delta_e = lastError - e;
            sumError += e;
            lastError = e;
            return (e * kP + delta_e * kD + sumError * kI);
        }
    }
}
