package frc.team10505.robot.subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team10505.robot.Constants.HardwareConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX leadMotor = new TalonFX(ELEVATOR_MOTOR_ID, "kingKan");
    public final TalonFX followMotor = new TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID, "kingKan");

    private double elevatorEncoderValue = 0.0;
    private double totalEffort;

    private double height = 0.0;
    private final static int kElevatorMotorCurrentLimit = 40;


    //PID and FeedFoward
    private PIDController elevatorController;
    private ElevatorFeedforward elevatorFeedforward;


    public ElevatorSubsystem() {
       
       
       var motorConfig = new MotorOutputConfigs();

       elevatorController = new PIDController(0.8, 0, 0);
       elevatorFeedforward = new ElevatorFeedforward(0, 0.4, 0, 0);
       
        //Set Current Limits
         var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = kElevatorMotorCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        leadMotor.getConfigurator().apply(motorConfig);
        leadMotor.getConfigurator().apply(limitConfigs);
    

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        followMotor.getConfigurator().apply(motorConfig);
        followMotor.getConfigurator().apply(limitConfigs);
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    }


    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public double getElevatorEncoder() {
        return (leadMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
    }

    public Boolean isNearGoal() {
        return MathUtil.isNear(height, getElevatorEncoder(), 2);
    }

    public boolean issGigh () {
        return getElevatorEncoder() > 30;
    }

    public Boolean isAbove(double heightOfChoice) {
        return getElevatorEncoder() > heightOfChoice;
    }

    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0, 0))
                + (elevatorController.calculate(getElevatorEncoder(), height)));
    }


    @Override

    public void periodic() {
        elevatorEncoderValue = getElevatorEncoder();
        totalEffort = getEffort();

        leadMotor.setVoltage(totalEffort * -1.0);

        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoderValue);
        SmartDashboard.putNumber("Elevator Effort", totalEffort);
        SmartDashboard.putNumber("Elevator Height", height);
        SmartDashboard.putBoolean("issGigh", issGigh());
        SmartDashboard.putNumber("Goooder", leadMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("goalHeight", height);
    }
    
    
    //TODO add motors(assign value in constructor. use if/else statement. in sim, dont have a canbus. irl we do)
    
    
    
    
    
    // TODO add other vars

    //TODO add constructor

    //TODO add methods

    //TODO add periodic mehtod and fill out

}
