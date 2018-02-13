/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Read Adafruit LIDAR sensor", group = "Example")
//@Disabled

public class ExampleI2cLIDARRead extends OpMode {
    int distance = 200;         //Variable for Distance data
    int lidarDistance = 0;      //Variable for combined sonar Distance reading
    int count = 0;
    int stop_variable;
    byte[] lidarCache;         //The read will return 14 bytes.  They are stored in this variable
    boolean read = false;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static final int LIDAR_ADDRESS = 0x52;                           //Default I2C address for Adafruit lidar
    public static final int LIDAR_READ_START = 0x14;                        //Register to start reading
    public static final int LIDAR_READ_LENGTH = 14;                         //Number of byte to read for distance
    public static final int VL53L0X_REG_SYSRANGE_START = 0x000;             //Register to issue commands to the sensor
    public static final int VL53L0X_REG_SYSRANGE_MODE_START_STOP = 0x01;    /** bit 0 in #VL53L0X_REG_SYSRANGE_START write 1 toggle state in
                                                                            * continuous mode and arm next shot in single shot mode */
    public static final int VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT = 0x00;    /** bit 1 write 0 in #VL53L0X_REG_SYSRANGE_START set single shot mode */
    public static final int VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK = 0x02;    /** bit 1 write 1 in #VL53L0X_REG_SYSRANGE_START set back-to-back
                                                                            *  operation mode */
    public static final int VL53L0X_SYS_INTR_CLR = 0x0B;
    public static final int VL53L0X_RESULT_INTR_STATUS =0x13;
    public static final int VL53L0X_MODEL_ID = 0xC0;                        //Sensor Model number should be 238
    public static final int VL53L0X_REVISION_ID = 0xC2;                     //Sensor firmware version should be 16
    public static final int VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x0089;

    public static final int VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N = 0x00bf;
    public static final int VL53L0X_REG_SYSRANGE_MODE_MASK = 0x0F;          //mask existing bit in #VL53L0X_REG_SYSRANGE_START*/


    public I2cAddr lidarAddr;
    public I2cDevice lidarSensor;
    public I2cDeviceSynch lidarSynch;

    /*DeviceModes DeviceMode;

    public enum DeviceModes {
        VL53L0X_DEVICEMODE_SINGLE_RANGING,
        VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
        VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM,
        VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING}*/


    @Override
    public void init() {
        lidarSensor = hardwareMap.i2cDevice.get("lidar");
        lidarAddr = I2cAddr.create8bit(LIDAR_ADDRESS);
        lidarSynch = new I2cDeviceSynchImpl(lidarSensor, lidarAddr, false);
        lidarSynch.engage();

        VL53LOX_DataInit();

        lidarSynch.write8(VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);

        telemetry.addData("Lidar Model (238)", lidarSynch.read8(VL53L0X_MODEL_ID)+ 256);
        telemetry.addData("Lidar firmware (16)", lidarSynch.read8(VL53L0X_REVISION_ID));
    }

    @Override
    public void init_loop() {
    telemetry.addData("Lidar Model (238)", lidarSynch.read8(VL53L0X_MODEL_ID)+ 256);
    telemetry.addData("Lidar firmware (16)", lidarSynch.read8(VL53L0X_REVISION_ID));
}

    @Override
    public void start() {
        lidarCache = lidarSynch.read(LIDAR_READ_START, LIDAR_READ_LENGTH);
        lidarSynch.write8(VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK + VL53L0X_REG_SYSRANGE_MODE_START_STOP);
        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.time() > 2000) {
            lidarCache = lidarSynch.read(LIDAR_READ_START, LIDAR_READ_LENGTH);
            timer.reset();
            count++;
            read = true;
        } else if (read) {
            lidarSynch.write8(VL53L0X_SYS_INTR_CLR, 0x01);
            read = false;
        }

        /*
	 * use multi read even if some registers are not useful, result will
	 * be more efficient
	 * start reading at 0x14 dec20
	 * end reading at 0x21 dec33 total 14 bytes to read
	 */

        //One byte only goes to 256 values.  Two bytes must be combined to get the total distance
        //lidarDistance = 0X000000FF & (int) lidarCache[0];
        //lidarDistance = (lidarDistance << 8 | (0X000000FF & (int) lidarCache[1]));

        //distance = lidarDistance > 1200 ? distance : lidarDistance;

        // send the info back to driver station using telemetry function.
        telemetry.addData("Timer", timer.toString());
        telemetry.addData("Cache length ", lidarCache.length);
        telemetry.addData("Lidar Model (238)", lidarSynch.read8(VL53L0X_MODEL_ID)+ 256);
        telemetry.addData("Interrupt", lidarSynch.read8(VL53L0X_SYS_INTR_CLR));
        telemetry.addData("Count", count);
        for(int i = 0; i < lidarCache.length; i++) {
            telemetry.addData("Byte " + i, lidarCache[i]);
        }
        //telemetry.addData("LSB " + expectedValue1, lidarCache[1]);
        //telemetry.addData("DIST", distance );
        //telemetry.addData("Read Register", LIDAR_READ_START);


    }


    @Override
    public void stop() {

    }

    public void VL53LOX_DataInit() {

	/* by default the I2C is running at 1V8 if you want to change it you
	 * need to include this define at compilation level. */
      //  lidarSynch.write8(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,0xFF);

	/* Set I2C standard mode */
        lidarSynch.write8(0x88,0x00);
    }


   /* public void VL53LOX_StartMeasurement() {
        byte Byte;
        byte StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
        uint32_t LoopNb;

	    Status = VL53L0X_WrByte(Dev, 0x80, 0x01);
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
        Status = VL53L0X_WrByte(Dev, 0x00, 0x00);
        Status = VL53L0X_WrByte(Dev, 0x91, PALDevDataGet(Dev, StopVariable));
        Status = VL53L0X_WrByte(Dev, 0x00, 0x01);
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);
        Status = VL53L0X_WrByte(Dev, 0x80, 0x00);

        switch (DeviceMode) {
            case VL53L0X_DEVICEMODE_SINGLE_RANGING:
                lidarSynch.write8(VL53L0X_REG_SYSRANGE_START, 0x01);

                Byte = StartStopByte;
                if (Status == VL53L0X_ERROR_NONE) {
			*//* Wait until start bit has been cleared *//*
                    LoopNb = 0;
                    do {
                        if (LoopNb > 0)
                            Status = VL53L0X_RdByte(Dev,
                                    VL53L0X_REG_SYSRANGE_START, &Byte);
                        LoopNb = LoopNb + 1;
                    } while (((Byte & StartStopByte) == StartStopByte)
                            && (Status == VL53L0X_ERROR_NONE)
                            && (LoopNb < VL53L0X_DEFAULT_MAX_LOOP));

                    if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
                        Status = VL53L0X_ERROR_TIME_OUT;

                }

                break;
            case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
		*//* Back-to-back mode *//*

		*//* Check if need to apply interrupt settings *//*
                if (Status == VL53L0X_ERROR_NONE)
                    Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);

                Status = VL53L0X_WrByte(Dev,
                        VL53L0X_REG_SYSRANGE_START,
                        VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
                if (Status == VL53L0X_ERROR_NONE) {
			*//* Set PAL State to Running *//*
                    PALDevDataSet(Dev, PalState, VL53L0X_STATE_RUNNING);
                }
                break;
            case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
		*//* Continuous mode *//*
		*//* Check if need to apply interrupt settings *//*
                if (Status == VL53L0X_ERROR_NONE)
                    Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);

                Status = VL53L0X_WrByte(Dev,
                        VL53L0X_REG_SYSRANGE_START,
                        VL53L0X_REG_SYSRANGE_MODE_TIMED);

                if (Status == VL53L0X_ERROR_NONE) {
			*//* Set PAL State to Running *//*
                    PALDevDataSet(Dev, PalState, VL53L0X_STATE_RUNNING);
                }
                break;
            default:
		*//* Selected mode not supported *//*
                Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
        }


        LOG_FUNCTION_END(Status);
        return Status;
    }

    public void VL53LOX_StopMeasurement() {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        LOG_FUNCTION_START("");

        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START,
                VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT);

        Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
        Status = VL53L0X_WrByte(Dev, 0x00, 0x00);
        Status = VL53L0X_WrByte(Dev, 0x91, 0x00);
        Status = VL53L0X_WrByte(Dev, 0x00, 0x01);
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);

        if (Status == VL53L0X_ERROR_NONE) {
		*//* Set PAL State to Idle *//*
            PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);
        }

	*//* Check if need to apply interrupt settings *//*
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 0);

        LOG_FUNCTION_END(Status);
        return Status;
    }

    public void VL53LOX_GetMeasurementDataReady() {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint8_t SysRangeStatusRegister;
        uint8_t InterruptConfig;
        uint32_t InterruptMask;
        LOG_FUNCTION_START("");

        InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
                Pin0GpioFunctionality);

        if (InterruptConfig ==
                VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
            Status = VL53L0X_GetInterruptMaskStatus(Dev, &InterruptMask);
            if (InterruptMask ==
                    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY)
			*pMeasurementDataReady = 1;
		else
			*pMeasurementDataReady = 0;
        } else {
            Status = VL53L0X_RdByte(Dev, VL53L0X_REG_RESULT_RANGE_STATUS,
                    &SysRangeStatusRegister);
            if (Status == VL53L0X_ERROR_NONE) {
                if (SysRangeStatusRegister & 0x01)
				*pMeasurementDataReady = 1;
			else
				*pMeasurementDataReady = 0;
            }
        }

        LOG_FUNCTION_END(Status);
        return Status;
    }

    public void VL53LOX_GetRangingMeasurementData() {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint8_t DeviceRangeStatus;
        uint8_t RangeFractionalEnable;
        uint8_t PalRangeStatus;
        uint8_t XTalkCompensationEnable;
        uint16_t AmbientRate;
        FixPoint1616_t SignalRate;
        uint16_t XTalkCompensationRateMegaCps;
        uint16_t EffectiveSpadRtnCount;
        uint16_t tmpuint16;
        uint16_t XtalkRangeMilliMeter;
        uint16_t LinearityCorrectiveGain;
        uint8_t localBuffer[12];
        VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

        LOG_FUNCTION_START("");

	*//*
	 * use multi read even if some registers are not useful, result will
	 * be more efficient
	 * start reading at 0x14 dec20
	 * end reading at 0x21 dec33 total 14 bytes to read
	 *//*
        Status = VL53L0X_ReadMulti(Dev, 0x14, localBuffer, 12);

        if (Status == VL53L0X_ERROR_NONE) {

            pRangingMeasurementData->ZoneId = 0; *//* Only one zone *//*
            pRangingMeasurementData->TimeStamp = 0; *//* Not Implemented *//*

            tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
		*//* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
		 *(format 11.2) else no fractional
		 *//*

            pRangingMeasurementData->MeasurementTimeUsec = 0;

            SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616(
                    VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6]));
		*//* peak_signal_count_rate_rtn_mcps *//*
            pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate;

            AmbientRate = VL53L0X_MAKEUINT16(localBuffer[9], localBuffer[8]);
            pRangingMeasurementData->AmbientRateRtnMegaCps =
                    VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);

            EffectiveSpadRtnCount = VL53L0X_MAKEUINT16(localBuffer[3],
                    localBuffer[2]);
		*//* EffectiveSpadRtnCount is 8.8 format *//*
            pRangingMeasurementData->EffectiveSpadRtnCount =
                    EffectiveSpadRtnCount;

            DeviceRangeStatus = localBuffer[0];

		*//* Get Linearity Corrective Gain *//*
            LinearityCorrectiveGain = PALDevDataGet(Dev,
                    LinearityCorrectiveGain);

		*//* Get ranging configuration *//*
            RangeFractionalEnable = PALDevDataGet(Dev,
                    RangeFractionalEnable);

            if (LinearityCorrectiveGain != 1000) {

                tmpuint16 = (uint16_t)((LinearityCorrectiveGain
                        * tmpuint16 + 500) / 1000);

			*//* Implement Xtalk *//*
                VL53L0X_GETPARAMETERFIELD(Dev,
                        XTalkCompensationRateMegaCps,
                        XTalkCompensationRateMegaCps);
                VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable,
                        XTalkCompensationEnable);

                if (XTalkCompensationEnable) {

                    if ((SignalRate
                            - ((XTalkCompensationRateMegaCps
                            * EffectiveSpadRtnCount) >> 8))
                            <= 0) {
                        if (RangeFractionalEnable)
                            XtalkRangeMilliMeter = 8888;
                        else
                            XtalkRangeMilliMeter = 8888
                                    << 2;
                    } else {
                        XtalkRangeMilliMeter =
                                (tmpuint16 * SignalRate)
                                        / (SignalRate
                                        - ((XTalkCompensationRateMegaCps
                                        * EffectiveSpadRtnCount)
                                        >> 8));
                    }

                    tmpuint16 = XtalkRangeMilliMeter;
                }

            }

            if (RangeFractionalEnable) {
                pRangingMeasurementData->RangeMilliMeter =
                        (uint16_t)((tmpuint16) >> 2);
                pRangingMeasurementData->RangeFractionalPart =
                        (uint8_t)((tmpuint16 & 0x03) << 6);
            } else {
                pRangingMeasurementData->RangeMilliMeter = tmpuint16;
                pRangingMeasurementData->RangeFractionalPart = 0;
            }

		*//*
		 * For a standard definition of RangeStatus, this should
		 * return 0 in case of good result after a ranging
		 * The range status depends on the device so call a device
		 * specific function to obtain the right Status.
		 *//*
            Status |= VL53L0X_get_pal_range_status(Dev, DeviceRangeStatus,
                    SignalRate, EffectiveSpadRtnCount,
                    pRangingMeasurementData, &PalRangeStatus);

            if (Status == VL53L0X_ERROR_NONE)
                pRangingMeasurementData->RangeStatus = PalRangeStatus;

        }

        if (Status == VL53L0X_ERROR_NONE) {
		*//* Copy last read data into Dev buffer *//*
            LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

            LastRangeDataBuffer.RangeMilliMeter =
                    pRangingMeasurementData->RangeMilliMeter;
            LastRangeDataBuffer.RangeFractionalPart =
                    pRangingMeasurementData->RangeFractionalPart;
            LastRangeDataBuffer.RangeDMaxMilliMeter =
                    pRangingMeasurementData->RangeDMaxMilliMeter;
            LastRangeDataBuffer.MeasurementTimeUsec =
                    pRangingMeasurementData->MeasurementTimeUsec;
            LastRangeDataBuffer.SignalRateRtnMegaCps =
                    pRangingMeasurementData->SignalRateRtnMegaCps;
            LastRangeDataBuffer.AmbientRateRtnMegaCps =
                    pRangingMeasurementData->AmbientRateRtnMegaCps;
            LastRangeDataBuffer.EffectiveSpadRtnCount =
                    pRangingMeasurementData->EffectiveSpadRtnCount;
            LastRangeDataBuffer.RangeStatus =
                    pRangingMeasurementData->RangeStatus;

            PALDevDataSet(Dev, LastRangeMeasure, LastRangeDataBuffer);
        }

        LOG_FUNCTION_END(Status);
        return Status;
    }

    public void VL53LOX_ClearInterruptMask() {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint8_t LoopCount;
        uint8_t Byte;
        LOG_FUNCTION_START("");

	*//* clear bit 0 range interrupt, bit 1 error interrupt *//*
        LoopCount = 0;
        do {
            Status = VL53L0X_WrByte(Dev,
                    VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
            Status |= VL53L0X_WrByte(Dev,
                    VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x00);
            Status |= VL53L0X_RdByte(Dev,
                    VL53L0X_REG_RESULT_INTERRUPT_STATUS, &Byte);
            LoopCount++;
        } while (((Byte & 0x07) != 0x00)
                && (LoopCount < 3)
                && (Status == VL53L0X_ERROR_NONE));


        if (LoopCount >= 3)
            Status = VL53L0X_ERROR_INTERRUPT_NOT_CLEARED;

        LOG_FUNCTION_END(Status);
        return Status;
    }

    public void VL53LOX_GetStopCompleteStatus() {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint8_t Byte = 0;
        LOG_FUNCTION_START("");

        Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);

        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_RdByte(Dev, 0x04, &Byte);

        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_WrByte(Dev, 0xFF, 0x0);

	*pStopStatus = Byte;

        if (Byte == 0) {
            Status = VL53L0X_WrByte(Dev, 0x80, 0x01);
            Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
            Status = VL53L0X_WrByte(Dev, 0x00, 0x00);
            Status = VL53L0X_WrByte(Dev, 0x91,
                    PALDevDataGet(Dev, StopVariable));
            Status = VL53L0X_WrByte(Dev, 0x00, 0x01);
            Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);
            Status = VL53L0X_WrByte(Dev, 0x80, 0x00);
        }

        LOG_FUNCTION_END(Status);
        return Status;
    }

    public void VL53LOX_GetRangeStatus() {}

*/

}