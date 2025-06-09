package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO import hardware constants AND coral constants, statically (instead of "import", write "import static" and before the semicolon, add ".*" to import all the vars in the class)

public class CoralSubsystem extends SubsystemBase {

        // Konstants
        private static final int kIntakeLeft = 1;//TODO move to constants
        private static final int kIntakeRight = 2;//TODO move to constants

        public static final int CORAL_MOTOR_CURRENT_LIMIT = 30;//TODO move to constants

        // Mystery Stuph defining
        private SparkMaxConfig coralLeftConfig = new SparkMaxConfig();
        private SparkMaxConfig coralRightConfig = new SparkMaxConfig();
        private final Mechanism2d flywheelHome = new Mechanism2d(5, 5);//TODO move(cut and paste to put it w/ other sim vars)
        // Encoders
        public double coralEncoderLeft = 0.0;//TODO delete. flywheels dont need feedback, ergo dont need encoders.
        public double coralEncoderRight = 0.0;//TODO delete. flywheels dont need feedback, ergo dont need encoders. 

        // Variables
        private SparkMax flywheelRight = new SparkMax(kIntakeRight, MotorType.kBrushless);
        private SparkMax flywheelLeft = new SparkMax(kIntakeLeft, MotorType.kBrushless);
        private MechanismLigament2d flywheelLeftVis = new MechanismLigament2d("Left Flywheel", kIntakeLeft,
                        CORAL_MOTOR_CURRENT_LIMIT, kIntakeLeft, null);//TODO delete, mech ligs are odd, and wont use their constructor like this. referense roughly 40 lines down to see how its done properly
        private MechanismLigament2d flywheelRightVis = new MechanismLigament2d("Right Flywheel", kIntakeRight,
                        CORAL_MOTOR_CURRENT_LIMIT, kIntakeRight, null);//TODO delete, mech ligs are odd, and wont use their constructor like this. referense roughly 40 lines down to see how its done properly(also why are there two??)

        // The Goods...
        public double getCoralEncoderL() {
                return (flywheelLeft.getAbsoluteEncoder().getPosition() * (Math.PI * 1.751 * 2) / 12.0) * 1.0;//TODO delete. lol is this from the seasons elevator? also methods belong BELOW the constructor
        }

        private final FlywheelSim intakeLeftSim = new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1)); // All the
                                                                                                              // numbers
                                                                                                              // (meaning
                                                                                                              // the
                                                                                                              // 0.005
                                                                                                              // and 5)
                                                                                                              // have
                                                                                                              // been
                                                                                                              // burgled
                                                                                                              // from
                                                                                                              // Motion
                                                                                                              // Magic
                                                                                                              // and Sim

        private final FlywheelSim intakeRightSim = new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1)); // All the
                                                                                                              // numbers
                                                                                                              // (meaning
                                                                                                              // the
                                                                                                              // 0.005
                                                                                                              // and 5)
                                                                                                              // have
                                                                                                              // been
                                                                                                              // burgled
                                                                                                              // from
                                                                                                              // Motion
                                                                                                              // Magic
                                                                                                              // and Sim

        private final MechanismRoot2d intakeLeftRoot = flywheelHome.getRoot("Intake Left Root", .6, 0.6); // Cords have
                                                                                                          // been
                                                                                                          // burgled too
                                                                                                          // :)
        private final MechanismRoot2d intakeRightRoot = flywheelHome.getRoot("Intake Right Root", 2.4, 0.6); // Cords
                                                                                                             // have
                                                                                                             // been
                                                                                                             // burgled
                                                                                                             // too :)


        private final MechanismLigament2d leftIntakeViz = intakeLeftRoot
                        .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000)); // All the numbers (meaning
                                                                                          // the 0.4 and 000) have been
                                                                                          // burgled from Motion Magic
                                                                                          // and Sim
        private final MechanismLigament2d rightIntakeViz = intakeRightRoot
                        .append(new MechanismLigament2d("rightIntakeLigament", 0.4, 180)); // All the numbers (meaning
                                                                                           // the 0.4 and 180) have been
                                                                                           // burgled from Motion Magic
                                                                                           // and Sim

        private double simMotorSpeed = 0.0;
        private double simMotorSpeed2 = 0.0;

        // Commands//TODO move BELOW the constructor
        public Command setFlywheelSpeed(double kflywheelSpeed) {//Name of the parameter doesn't need to start w k. (k is for konstants, and konstants only)
                if (Utils.isSimulation()) {
                        return runOnce(() -> {
                                simMotorSpeed = kflywheelSpeed;
                        });
                } else {
                        return runOnce(() -> {
                                flywheelLeft.set(kflywheelSpeed);
                                flywheelRight.set(kflywheelSpeed);
                        });

                }
        }

        // Konstructor
        public CoralSubsystem() {
                coralLeftConfig.idleMode(IdleMode.kBrake);
                coralRightConfig.idleMode(IdleMode.kBrake);
                coralLeftConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT);
                coralRightConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT);
                flywheelLeft.configure(coralLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                flywheelRight.configure(coralRightConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                //TODO add smartdashboard thing to put mechanism2d (reference personal project if needed)

        }

        @Override
        public void periodic() {//TODO fill out

                if (Utils.isSimulation()) {

                } else {

                }

        }

}
