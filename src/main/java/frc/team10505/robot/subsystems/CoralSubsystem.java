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
        

        // Variables
        private SparkMax flywheelRight = new SparkMax(kIntakeRight, MotorType.kBrushless);
        private SparkMax flywheelLeft = new SparkMax(kIntakeLeft, MotorType.kBrushless);                                                         
                                                                                


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


}
