package frc.team10505.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//TODO import hardware constants AND algae constants, statically (instead of "import", write "import static" and before the semicolon, add ".*" to import all the vars in the class)

public class AlgaeSubsystem extends SubsystemBase {

    private final int kAlgaeIntakeMotorID = 7;//TODO move to constants
    private final int kAlgaePivotMotorID = 8;//TODO move to constants

    private final double startingAngle = 0.0;

    /* Intake Speeds *///TODO move all intake speeds to constants
    private double intakeSpeed = 15;
    private double intakeSpeedSlow = 10;
    private double intakeStop = 0;
    private double simMotorSpeed = 30;
    private double simSpeed = 0;

    //TODO add constructor

    //TODO add methods

    //TODO add periodic method and fill out

}
