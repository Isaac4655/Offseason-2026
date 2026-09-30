package team7111.robot.utils.motor;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.invoke.ClassSpecializer.Factory;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.utils.encoder.GenericEncoder;

public class NovaMotor implements Motor {
    private ThriftyNova motor;
    private PIDController pid = new PIDController(0.05, 0, 0);
    private ThriftyNovaConfig novaConfig;
    private MechanismType mechanismType;
    private GenericEncoder encoder = null;
    private double gearRatio;
    private double currentSetpoint;
    private double velocitySetpoint = 0;
    private double positiveVoltageLimit = 100;
    private double negativeVoltageLimit = -100;
    private double positiveVelocityLimit = 5000;
    private double negativeVelocityLimit = -5000;
    private Follower follower;
    private SimpleMotorFeedforward feedforward;
    private ArmFeedforward armFF;
    private ElevatorFeedforward elevatorFF;
    private int id;
    private boolean isFollower = false;
    private boolean isFollowerInverted = false;
    private int leaderID;

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    private GenericEntry motorPEntry;
    private GenericEntry motorIEntry;
    private GenericEntry motorDEntry;

    public NovaMotor(int id, GenericEncoder encoder, MotorConfig config){
        this.encoder = encoder;
        this.gearRatio = config.gearRatio;
        this.pid = config.pid;
        this.feedforward = config.simpleFF;
        this.armFF = config.armFF;
        this.elevatorFF = config.elevatorFF;
        this.id = id;
        motor = new ThriftyNova(id);
        motorPEntry = Shuffleboard.getTab("test").add("Motor " + id + " P", 0).getEntry();
        motorIEntry = Shuffleboard.getTab("test").add("Motor " + id + " I", 0).getEntry();
        motorDEntry = Shuffleboard.getTab("test").add("Motor " + id + " D", 0).getEntry();

        config.novaConfig.pid0.p = pid.getP();
        config.novaConfig.pid0.i = pid.getI();
        config.novaConfig.pid0.d = pid.getD();
        config.novaConfig.pid0.f = feedforward.getKs();
        // Two more feed forward configs are on the original CTREMotor, that I could not find for here.
        config.novaConfig.maxCurrent = (double) config.currentLimit;
        config.novaConfig.inverted = config.isInverted;
        config.novaConfig.brakeMode = config.isBreakMode;
        motor.applyConfig(novaConfig);
    }

    public NovaMotor(int id){
        this.id = id;
        this.mechanismType = MechanismType.flywheel;
        motor = new ThriftyNova(id);
        motorPEntry = Shuffleboard.getTab("test").add("Motor " + id + " P", 0).getEntry();
        motorIEntry = Shuffleboard.getTab("test").add("Motor " + id + " I", 0).getEntry();
        motorDEntry = Shuffleboard.getTab("test").add("Motor " + id + " D", 0).getEntry();

        setPositionReadout(0);
    }

    public void setDutyCycle(double speed) {
        motor.set(speed);
    }

    public double getDutyCycle(){
        return motor.get();
    }

    public void setVelocity(double rpm){
        
        if(rpm > positiveVelocityLimit){
            rpm = positiveVelocityLimit;
        }
        if(rpm < negativeVelocityLimit){
            rpm = negativeVelocityLimit;
        }
        velocitySetpoint = rpm * gearRatio;

        rpm /= 60;
        motor.setVelocity(rpm * gearRatio);
    }

    public double getVelocity() {
        return motor.getVelocity() * 60 / gearRatio;
    }

    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    public void setPositionReadout(double position) {
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        } else {
            motor.setPosition(Degrees.of(position).in(Rotations) * gearRatio);
        }
    }

    public double getPosition() {
        if(encoder == null){
            return motor.getPosition() / gearRatio * 360;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }

    public void setSetpoint(double setPoint, boolean useFF) {
        currentSetpoint = setPoint;
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput = 0;
        
        if(useFF){
            switch(mechanismType){
                case arm:
                    feedforwardOutput = armFF.calculate(Degrees.of(setPoint).in(Radians), pid.getErrorDerivative());
                    break;
                case elevator:
                    feedforwardOutput = elevatorFF.calculate(pid.getErrorDerivative());
                    break;
                case flywheel:
                default:
                    feedforwardOutput = feedforward.calculate(pid.getErrorDerivative());
                    break;
            }
        }

        double totalOutput = pidOutput + feedforwardOutput;
        SmartDashboard.putNumber("Motor " + id + " pid", totalOutput);
        if(totalOutput > positiveVoltageLimit){
            motor.setVoltage(positiveVoltageLimit);
            //motor.set(positiveVoltageLimit);
        }else if(totalOutput < negativeVoltageLimit){
            motor.setVoltage(negativeVoltageLimit);
            //motor.set(negativeVoltageLimit);
        }else{
            motor.setVoltage(totalOutput);
        }
        SmartDashboard.putNumber("Motor " + id + " pid output", totalOutput);
    }

    public void setPID(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    public PIDController getPID() {
        return pid;
    }

    public GenericEncoder getEncoder() {
        return encoder;
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public boolean isAtSetpoint(double deadzone) {
        if(getPosition() >= currentSetpoint - deadzone && getPosition() <= currentSetpoint + deadzone){
            return true;
        }
        return false;
    }

    public boolean isAtVelocitySetpoint(double deadZone) {
        return getVelocity() >= velocitySetpoint - deadZone && getVelocity() <= velocitySetpoint + deadZone;
    }

    public SimpleMotorFeedforward getFeedForward() {
        return feedforward;
    }

    /*
    * Nova config only takes a single static feed forward (kS) additional ff values found in Nova profiler
    * Nova configs PIDConfiguration takes P, I, D, and F, feed forward is not a seperate object, and cannot be expanded.
    */

    public void setFeedFoward(double kS, double kV, double kA) {
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void periodic() {
        if (encoder != null){
            encoder.periodic();
        }
        SmartDashboard.putNumber("Motor " + id + " setpoint", currentSetpoint);
        SmartDashboard.putBoolean("Motor " + id + " isAtSetpoint", isAtSetpoint(0.05));
    }

    public void setFollower(boolean isFollower, int id, boolean isInverted){
        if(isFollower && !this.isFollower){
            motor.follow(id);
            motor.factoryReset();
            motor.applyConfig(novaConfig);
        }
        this.isFollower = isFollower;
    }

    public void setSpeedLimits(double positiveSpeed, double negativeSpeed, boolean isVoltage) {
        if(isVoltage){
            positiveVoltageLimit = positiveSpeed;
            negativeVoltageLimit = negativeSpeed;
        }else{
            positiveVelocityLimit = positiveSpeed;
            negativeVelocityLimit = negativeSpeed;
        }
    }

    
}
