/**
 * This software is free to use and to distribute in its unchanged form for private use.
 * Commercial use is prohibited without an explicit license agreement of the copyright holder.
 * Any changes to this software must be made solely in the project repository at https://github.com/ai-republic/bms-to-inverter.
 * The copyright holder is not liable for any damages in whatever form that may occur by using this software.
 *
 * (c) Copyright 2022 and onwards - Torsten Oltmanns
 *
 * @author Torsten Oltmanns - bms-to-inverter''AT''gmail.com
 */
package com.airepublic.bmstoinverter.inverter.pylon.rs485;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

import javax.enterprise.context.ApplicationScoped;
import javax.inject.Inject;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airepublic.bmstoinverter.core.Inverter;
import com.airepublic.bmstoinverter.core.Port;
import com.airepublic.bmstoinverter.core.bms.data.BatteryPack;
import com.airepublic.bmstoinverter.core.bms.data.EnergyStorage;
import com.airepublic.bmstoinverter.core.bms.data.EnergyStorageQualifier;

@ApplicationScoped
public class Pylon2InverterRS485Processor extends Inverter {
    private final static Logger LOG = LoggerFactory.getLogger(Pylon2InverterRS485Processor.class);

    @Inject
    @EnergyStorageQualifier
    private EnergyStorage energyStorage;

    public Pylon2InverterRS485Processor() {
        super();
    }


    protected Pylon2InverterRS485Processor(final EnergyStorage energyStorage) {
        super(energyStorage);
        this.energyStorage = energyStorage;
    }


    @Override
    protected List<ByteBuffer> createSendFrames(final ByteBuffer requestFrame, final BatteryPack aggregatedPack) {
        final List<ByteBuffer> frames = new ArrayList<>();

        double sumSOC = 0;
        double sumVoltage = 0;
        int count = 0;

        // Settings for charging current (Amperes)
        final int targetLimitAmps = 100;
        final int balanceLimitAmps = 32; // Inverter will throttle this to ~8A actual charge at high SOC

        if (energyStorage != null && !energyStorage.getBatteryPacks().isEmpty()) {
            for (final BatteryPack pack : energyStorage.getBatteryPacks()) {
                if (pack != null && pack.packVoltage > 100) {
                    final double currentSoc = pack.packSOC > 100 ? pack.packSOC / 10.0 : pack.packSOC;
                    sumSOC += currentSoc;
                    sumVoltage += pack.packVoltage;
                    count++;
                }
            }
        }

        if (count == 0) {
            return frames;
        }

        final int avgSoc = (int) Math.round(sumSOC / count);
        final double avgVolt = sumVoltage / count;

        // Adaptive limit: 100A below 98% SOC, 32A at or above 98% SOC
        final int currentAmps = avgSoc >= 98 ? balanceLimitAmps : targetLimitAmps;

        // SCALE 100 (Based on ESP32 logic: 100A * 100 = 10000 / 32A * 100 = 3200)
        final short rawLimit = (short) (currentAmps * 100);

        final ByteBuffer buffer = ByteBuffer.allocate(43).order(ByteOrder.BIG_ENDIAN);
        buffer.put((byte) 0x01); // Slave ID
        buffer.put((byte) 0x03); // Function
        buffer.put((byte) 0x26); // Data Length 38

        // [Offset 3-4] Voltage (V * 10)
        buffer.putShort((short) Math.round(avgVolt));

        // [Offset 5-13] Padding (9 bytes)
        for (int i = 0; i < 9; i++) {
            buffer.put((byte) 0x00);
        }

        // [Offset 14] SOC (1 byte)
        buffer.put((byte) avgSoc);

        // [Offset 15-20] Padding (6 bytes)
        for (int i = 0; i < 6; i++) {
            buffer.put((byte) 0x00);
        }

        // [Offset 21-22] Charge Current Limit (Scale 100)
        buffer.putShort(rawLimit);

        // [Offset 23-24] Discharge Current Limit (Scale 100)
        buffer.putShort((short) (targetLimitAmps * 100));

        // Padding until CRC
        while (buffer.position() < 41) {
            buffer.put((byte) 0x00);
        }

        final byte[] data = buffer.array();
        final int crc = calculateCRC(data, 41);
        buffer.put((byte) (crc & 0xFF));
        buffer.put((byte) (crc >> 8 & 0xFF));

        buffer.rewind();
        frames.add(buffer);

        LOG.info("Bridge -> Inverter: SOC={}%, TargetLimit={}A, SentRaw={}, Count={}",
                avgSoc, currentAmps, rawLimit, count);

        return frames;
    }


    @Override
    protected ByteBuffer readRequest(final Port port) throws IOException {
        return port.receiveFrame();
    }


    @Override
    protected void sendFrame(final Port port, final ByteBuffer frame) throws IOException {
        port.sendFrame(frame);
    }


    private int calculateCRC(final byte[] buf, final int len) {
        int crc = 0xFFFF;
        for (int pos = 0; pos < len; pos++) {
            crc ^= buf[pos] & 0xFF;
            for (int i = 8; i != 0; i--) {
                if ((crc & 0x0001) != 0) {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
}