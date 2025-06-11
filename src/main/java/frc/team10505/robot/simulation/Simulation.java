package frc.team10505.robot.simulation;

import static frc.team10505.robot.Constants.HardwareConstants.ALGAE_INTAKE_MOTOR_ID;
import static frc.team10505.robot.Constants.HardwareConstants.ALGAE_PIVOT_MOTOR_ID;
import static frc.team10505.robot.Constants.HardwareConstants.CORAL_IN_LASER_ID;
import static frc.team10505.robot.Constants.HardwareConstants.CORAL_LEFT_MOTOR_ID;
import static frc.team10505.robot.Constants.HardwareConstants.CORAL_RIGHT_MOTOR_ID;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.Constants.HardwareConstants.*;

public class Simulation extends SubsystemBase {
    private AlgaeSubsystem algaeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private CoralSubsystem coralSubsystem;
    
    public double pivotPos;
    public double simPivotPos;

    // colours
    private final Color8Bit gween = new Color8Bit(Color.kGreen);
    private final Color8Bit owange = new Color8Bit(Color.kOrange);
    private final Color8Bit Purwpuwl = new Color8Bit(Color.kPurple);

    // mechanisms
    private final Mech pivotMech = new Mech("Algae", 5, 5);
    private final Mech coralIntakeMech = new Mech("Coral", 5, 5);

    // Sims

    private final FlywheelSim coralIntakesim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.00001, 4), DCMotor.getKrakenX60(1));
    private final FlywheelSim pivIntakesim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.00001, 4), DCMotor.getKrakenX60(1));

    public Simulation(AlgaeSubsystem algaeSubsys, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        this.algaeSubsystem = algaeSubsys;
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        pivotMech.addViz("Pivot", 2.5, 2.5, 3, 0, 10, Purwpuwl);
        pivotMech.addViz2("Pivot Intake", 5, 5, 1, 0, 10, gween);
        coralIntakeMech.addViz("lIntake", 1.25, 2.5, 1, 0, 10, owange);
        coralIntakeMech.addViz2("rIntake", 3.75, 2.5, 1, -180, 10, owange);

        SmartDashboard.putData("pivotMech", pivotMech.getMech());
        SmartDashboard.putData("coralIntakeMech", coralIntakeMech.getMech());
    }

    @Override
    public void simulationPeriodic() {
        coralIntakeMech.viz.setAngle(coralIntakeMech.viz.getAngle() + (coralIntakesim.getAngularVelocityRPM() * 2), new Rotation3d(0, coralIntakeMech.viz.getAngle(), 0));
        coralIntakeMech.viz2.setAngle(coralIntakeMech.viz2.getAngle() + (coralIntakesim.getAngularVelocityRPM() * 2), new Rotation3d(0, coralIntakeMech.viz2.getAngle(), 0));

        pivotMech.viz.setAngle(algaeSubsystem.getPivotEncoder(),
                new Rotation3d(0, algaeSubsystem.getPivotEncoder(), 0));
        pivotMech.viz2.setAngle(pivotMech.viz2.getAngle() + (pivIntakesim.getAngularVelocityRPM() * 2),
                new Rotation3d(0, pivotMech.viz2.getAngle(), 0));
        pivotMech.viz2.setRoot((Math.cos(Units.degreesToRadians(pivotMech.viz.getAngle()))*0.56) + 0.75, (Math.sin(Units.degreesToRadians(pivotMech.viz.getAngle())) * 0.56) + 0.75);

       pivotMech.viz.updatePublisher();
       pivotMech.viz2.updatePublisher();
       coralIntakeMech.viz.updatePublisher();
       coralIntakeMech.viz2.updatePublisher();
    }

}
