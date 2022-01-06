package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_P_Controller {
    // this gives us access to all opMode information
    private LinearOpMode opMode;

    private double setpoint = 0;
    private double minInput = 0;
    private double maxInput = 0;
    private double minOutput = 0;
    private double maxOutput = 0;
    private double outputRange = 0;
    private double inputRange = 0;
    private double absError = 0;
    private double currentError = 0;
    private double currentInput = 0;

    private final double PCT_THRESHOLD;
    private final double P_COEFFICIENT;

    public MM_P_Controller(LinearOpMode opMode, double pctThreshold, double pCoefficient){
        this.opMode = opMode;

        PCT_THRESHOLD = pctThreshold;//something
        P_COEFFICIENT = pCoefficient;//also something
    }
    public double calculatePower(double currentInput){
        this.currentInput = currentInput;
        absError = Math.abs(Math.abs(setpoint) - Math.abs(currentInput));
        currentError = setpoint - currentInput;

        double power = absError * P_COEFFICIENT * (outputRange);
        if(power > outputRange){
            power = outputRange;
        }
        opMode.telemetry.addData("input", currentInput);
        opMode.telemetry.addData("current error", absError);
        opMode.telemetry.addData("calculated power", power);
        return power;
    }
    public boolean reachedTarget(){
//        if ((absError / inputRange) * 100 < PCT_THRESHOLD){
        if ((Math.abs(currentError) / inputRange) * 100 < PCT_THRESHOLD){

        return true;
        }
        return false;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        inputRange = maxInput - minInput;
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        outputRange = maxOutput - minOutput;
    }

    public double getCurrentError(){
        return currentError;
    }

    public double getCurrentInput(){
        return currentInput;
    }
    public double getMinOutput(){
        return minOutput;
    }
}