package frc.team10505.robot;

import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;

public class Superstructure {
    // TODO add subsystems, but DON'T ASSIGN THEM A VALUE YET
    public AlgaeSubsystem algaeSubsys;
    public CoralSubsystem coralSubsys;
    public ElevatorSubsystem elevatorSubsys;
    public DrivetrainSubsystem drivetrainSubsys;

    // TODO add constructor
    public Superstructure(AlgaeSubsystem algaeSubsys, CoralSubsystem coralSubsys, ElevatorSubsystem elevatorSubsys,
            DrivetrainSubsystem drivetrainSubsys) {
        this.algaeSubsys = algaeSubsys;
        this.coralSubsys = coralSubsys;
        this.elevatorSubsys = elevatorSubsys;
        this.drivetrainSubsys = drivetrainSubsys;
    }

    // TODO add methods (will be commands)

    // TODO add auton compatable methods where needed
}
