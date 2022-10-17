package frc.lib.utils;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class CANCoderUtil {
    public enum CANCoderUsage {
        kAll,
        kSensorDataOnly,
        kFaultsOnly,
        kMinimal
    }

    public static void setCANCoderBusUsage(WPI_CANCoder encoder, CANCoderUsage usage) {
        if (usage == CANCoderUsage.kAll) {
            encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
            encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
        } else if (usage == CANCoderUsage.kSensorDataOnly) {
            encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
            encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
        } else if (usage == CANCoderUsage.kFaultsOnly) {
            encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
            encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
        } else if (usage == CANCoderUsage.kMinimal) {
            encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
            encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
        }
    }
}
