package org.firstinspires.ftc.robotcontroller.internal;

import java.util.ArrayList;

public class PID {
    public PID(double targetValue, double kp, double ki, double kd) {
        this.targetValue = targetValue;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public PID(double targetValue) {
        this.targetValue = targetValue;
    }
    public PID() {}


    private double kp = 0.010; //proportionality constant (amount to adjust for immediate deviance) must be experimentally found
    private double ki = 0.001; //integral constant (amount to adjust for past errors) must be experimentally found
    private double kd = 0.0022; //derivative constant (amount to adjust for future errors) must be experimentally found
    private double targetValue = 0;
    private double error = 0;

    private ArrayList<Double> e_list = new ArrayList<>(); //records past errors
    private ArrayList<Double> t_list = new ArrayList<>(); // records times past errors took place


    public void setConstants(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public void setTargetValue(double targetValue) {
        this.targetValue = targetValue;
    }

    public void update(double currentValue, double time) {
        error = targetValue - currentValue;
        e_list.add(error);
        t_list.add(time);
    }
    public void reset() {
        e_list.clear();
        t_list.clear();
    }

    //integrates error of angle w/ respect to time
    private double integrate() {
        double sum = 0; //uses trapezoidal sum approximation method
        if (e_list.size() >= 2) {
            for (int i = 0; i <= e_list.size() - 2; i++) {
                double dt = t_list.get(i+1) - t_list.get(i);
                sum += (e_list.get(i+1) + e_list.get(i))*dt / 2.0;
            }
        }
        return sum;
    }
    //differentiates error of angle w/ respect to time
    private double differentiate() {
        double slope = 0; //uses secant line approximation
        if (e_list.size() >= 2) {
            double de = e_list.get(e_list.size() - 1) - e_list.get(e_list.size() - 2);
            double dt = t_list.get(t_list.size() - 1) - t_list.get(t_list.size() - 2);
            slope = de/dt;
        }
        return slope;
    }

    public double adjustmentValue() {
        return kp*error + ki*integrate() + kd*differentiate();
    }
    public double getError() {
        return error;
    }
}
