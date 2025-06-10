package frc.team10505.robot.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;

public class Simulation extends SubsystemBase{
    private AlgaeSubsystem algaeSubsystem; 
    private ElevatorSubsystem elevatorSubsystem;
    private CoralSubsystem coralSubsystem;

    public Simulation(AlgaeSubsystem algaeSubsys, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        this.algaeSubsystem = algaeSubsys;
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }
    
}
