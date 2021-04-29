package org.firstinspires.ftc.teamcode;


public class PIDController {

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    
    private double error = 0;
    private double lastError = 0;
    private double totalError = 0;
    private double velocityError = 0;

    private double tolerance = 0;
    private double velocityTolerance = 0;

    private double lastTime = 0;
    private double setpoint = 0;
    
    private boolean continuous = false;
    private double minInput = 0;
    private double maxInput = 0;
    
    private boolean hasRun = false; // so we don't immediately return true
    
    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, false);
    }
    
    public PIDController(double kP, double kI, double kD, double minInput, double maxInput, boolean continuous) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        
        this.continuous = continuous;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }
    
    public double inputModulus(double input) {
        double modulus = maxInput - minInput;
        
        int numMax = (int) ((input - minInput) / modulus);
        input -= numMax * modulus;
        
        int numMin = (int) ((input - maxInput) / modulus);
        input -= numMin * modulus;
        
        return input;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double measurement, double setpoint, double time) {
        setSetpoint(setpoint);
        return calculate(measurement, time);
    }

    public double calculate(double measurement, double time) {
        hasRun = true;
        if (continuous) {
            error = inputModulus(setpoint - measurement);
        } else {
            error = setpoint - measurement;
        }

        velocityError = (error - lastError) / (time - lastTime);
        //.velocityError = 0;
        totalError += error;

        double output = kP * error + kI * totalError + kD * velocityError;
        lastError = error;
        lastTime = time;

        return output;
    }
    
    public void setTolerance(double tolerance, double velocityTolerance) {
        this.tolerance = tolerance;
        this.velocityTolerance = velocityTolerance;
    }

    public void reset() {
        error = 0;
        lastError = 0;
        totalError = 0;
        velocityError = 0;
        lastTime = 0;
    }
    
    public double getError() {
        return error;
    }
    
    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        if (hasRun) {
            return Math.abs(error) < tolerance && Math.abs(velocityError) < velocityTolerance;
        }
        return false;
    }
}