package frc.team10505.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import static frc.team10505.robot.Constants.HardwareConstants.*;
import static frc.team10505.robot.Constants.CoralConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
        // Variables (Motors)
        private SparkMax flywheelRight = new SparkMax(CORAL_RIGHT_MOTOR_ID, MotorType.kBrushless);
        private SparkMax flywheelLeft = new SparkMax(CORAL_LEFT_MOTOR_ID, MotorType.kBrushless);
        private final LaserCan inlaser = new LaserCan(CORAL_IN_LASER_ID);
        private final LaserCan outLaser = new LaserCan(CORAL_OUT_LASER_ID);

        // Mystery Stuph defining (Configurators)
        private SparkMaxConfig coralLeftConfig = new SparkMaxConfig();
        private SparkMaxConfig coralRightConfig = new SparkMaxConfig();

        // constructor
        public CoralSubsystem() {
                coralLeftConfig.idleMode(IdleMode.kBrake);
                coralRightConfig.idleMode(IdleMode.kBrake);
                coralLeftConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT);
                coralRightConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT);
                flywheelLeft.configure(coralLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                flywheelRight.configure(coralRightConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        public boolean inSensor() {
                LaserCan.Measurement inMeas = inlaser.getMeasurement();
                return (inMeas.distance_mm < 50 && inMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
        }
        public boolean outSensor() {
                LaserCan.Measurement outMeas = outLaser.getMeasurement();
                return (outMeas.distance_mm < 100 && outMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
        }

        
        public Command Intake() {
                return runEnd(() -> {
                        flywheelLeft.set(kIntakeSpeed);
                        flywheelRight.set(kIntakeSpeed);
                }, () -> {
                        flywheelLeft.set(0);
                        flywheelRight.set(0.0);
                        
                });}
        
        public Command Outtake() {
                return runEnd(() -> {
                        flywheelLeft.set(kOutakeSpeed);
                        flywheelRight.set(kOutakeSpeed);
                }, () -> {
                        flywheelLeft.set(0);
                        flywheelRight.set(0);
                });
        }

        public Command setOutake() {
                return run(() -> {
                        flywheelLeft.set(kOutakeSpeed);
                        flywheelRight.set(kOutakeSpeed);
                });
        }
        
        public Command outakeTop() {
                return runEnd(() -> {
                        flywheelLeft.set(kOutakeTopSpeed);
                        flywheelRight.set(kOutakeTopSpeed);
                }, () -> {
                        flywheelLeft.set(0);
                        flywheelRight.set(0);
                });
        }

        public Command setOutakeTop() {
                return run(() -> {
                        flywheelLeft.set(kOutakeTopSpeed);
                        flywheelRight.set(kOutakeTopSpeed);
                });
        }
        
        
        public Command trough() {
                return runEnd(() -> {
                        flywheelLeft.set(kLeftL1Speed);
                        flywheelRight.set(kRightL1Speed);
                }, () -> {
                        flywheelLeft.set(0);
                        flywheelRight.set(0);
                });
        }

        public Command stop() {
                return run(() -> {
                        flywheelLeft.set(0.0);
                        flywheelRight.set(0.0);
                });
        }

        public Command setstop() {
                return run(() -> {
                        flywheelLeft.set(0.0);
                        flywheelRight.set(0.0);
                });
        }

        public Command Slow() {
                return runEnd(() -> {
                        flywheelLeft.set(0.05);
                        flywheelRight.set(0.05);
                }, () -> {
                        flywheelLeft.set(0.0);
                        flywheelRight.set(0.0);
                });
        }

        public Command autoIntake() {
                return runEnd(() -> {
                        flywheelLeft.set(kIntakeSpeed);
                        flywheelRight.set(kIntakeSpeed);
                }, () -> {
                        Slow().until(() -> (outSensor() && !inSensor()));
                });
        }

        public Command autoSetIntake() {
                return run(() -> {
                        flywheelLeft.set(kIntakeSpeed);
                        flywheelRight.set(kIntakeSpeed);
                });
        }

        @Override
        public void periodic() {
                SmartDashboard.putBoolean("Insensor", inSensor());
                SmartDashboard.putBoolean("Outsensor", outSensor());
        }
       
}


