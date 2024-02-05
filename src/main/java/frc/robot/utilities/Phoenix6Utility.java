package frc.robot.utilities;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;


// TODO Move used code to swerve module
public final class Phoenix6Utility {

    private static final int CONFIG_RETRY_COUNT = 5;

    public static StatusCode resetTalonFxFactoryDefaults(TalonFX talon) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        return setTalonFxConfiguration(talon, cfg);
    }

    public static StatusCode setTalonFxConfiguration(TalonFX talon, TalonFXConfiguration cfg, boolean resetToFactoryDefaults) {
        if(resetToFactoryDefaults) {
            resetTalonFxFactoryDefaults(talon);
            // Don't return if fails since still want to set configuration!
        }
         return applyConfigAndRetry(talon, () -> talon.getConfigurator().apply(cfg));
    }
    private static StatusCode setTalonFxConfiguration(TalonFX talon, TalonFXConfiguration cfg) {
        return setTalonFxConfiguration(talon, cfg, false);
    }


    public static StatusCode resetCANcoderFactoryDefaults(CANcoder canCoder) {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        return setCANcoderConfiguration(canCoder, cfg);
    }

    private static StatusCode setCANcoderConfiguration(CANcoder canCoder, CANcoderConfiguration cfg, boolean resetToFactoryDefaults) {
        if(resetToFactoryDefaults) {
            resetCANcoderFactoryDefaults(canCoder);
            // Don't return if fails since still want to set configuration!
        }
        return applyConfigAndRetry(canCoder, () -> canCoder.getConfigurator().apply(cfg));
    }
    private static StatusCode setCANcoderConfiguration(CANcoder canCoder, CANcoderConfiguration cfg) {
        return setCANcoderConfiguration(canCoder, cfg, false);
    }

    public static StatusCode resetPigeon2FactoryDefaults(Pigeon2 pigeon2) {
        Pigeon2Configuration cfg = new Pigeon2Configuration();
        return setPigeon2Configuration(pigeon2, cfg);
    }

    private static StatusCode setPigeon2Configuration(Pigeon2 pigeon2, Pigeon2Configuration cfg, boolean resetToFactoryDefaults) {
        if(resetToFactoryDefaults) {
            resetPigeon2FactoryDefaults(pigeon2);
            // Don't return if fails since still want to set configuration!
        }

        return applyConfigAndRetry(pigeon2, () -> pigeon2.getConfigurator().apply(cfg));
    }
    private static StatusCode setPigeon2Configuration(Pigeon2 pigeon2, Pigeon2Configuration cfg) {

        return setPigeon2Configuration(pigeon2, cfg, false);
    }


    /**
     * Apply Configure to Ctre device and validate configuration is applied (retry if fails)
     * @param device
     * @param toApply
     */
    public static StatusCode applyConfigAndRetry(ParentDevice device, Supplier<StatusCode> toApply) {
        return applyConfig(device, toApply, false);
    }
    /**
     * Apply Configure to Ctre device and validate configuration is applied (no retry if fails)
     * @param device
     * @param toApply
     */
    public static StatusCode applyConfigAndNoRetry(ParentDevice device, Supplier<StatusCode> toApply) {
        return applyConfig(device, toApply, true);
    }

    public static StatusCode applyConfig(ParentDevice device, Supplier<StatusCode> toApply, boolean noRetry) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = noRetry ? 1 : CONFIG_RETRY_COUNT;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        if(finalCode.isOK()) {
            DriverStation.reportError(
                String.format("Unable to configure device %s: %s", device.getDeviceID(), finalCode.toString()), 
                true);
        }
        return finalCode;
    }

}