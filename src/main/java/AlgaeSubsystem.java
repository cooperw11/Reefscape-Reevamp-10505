import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    private final int kAlgaeIntakeMotorID = 7;
    private final int kAlgaePivotMotorID = 8;

    private final double startingAngle = 0.0;

    /* Intake Speeds */
    private double intakeSpeed = 15;
    private double intakeSpeedSlow = 10;
    private double intakeStop = 0;
    private double simMotorSpeed = 30;
    private double simSpeed = 0;

}
