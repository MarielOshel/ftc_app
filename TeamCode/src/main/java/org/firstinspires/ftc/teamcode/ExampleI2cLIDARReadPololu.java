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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Read Adafruit LIDAR sensor Pololu", group = "Example")
//@Disabled

public class ExampleI2cLIDARReadPololu extends OpMode {

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static final int LIDAR_ADDRESS = 0x52;                           //Default I2C address for Adafruit lidar
    public static final int LIDAR_READ_START = 0x14;                        //Register to start reading
    public static final int LIDAR_READ_LENGTH = 14;                         //Number of byte to read for distance

    public I2cAddr lidarAddr;
    public I2cDevice lidarSensor;
    public I2cDeviceSynch lidarSynch;
    public byte sensorStatus; // measurement status of vl53lox
    public int timeout = 500;            //Maximum time to read the sensor
    public int count =0;
    public int readCount =0;
    public int loopCount =0;
    public int distance = 23000;
    public boolean isInitialized = false;

    // register addresses from API vl53l0x_device.h (ordered as listed there)
    public static final int SYSRANGE_START = 0x00;
    public static final int SYSTEM_THRESH_HIGH = 0x0C;
    public static final int SYSTEM_THRESH_LOW = 0x0E;
    public static final int SYSTEM_SEQUENCE_CONFIG = 0x01;
    public static final int SYSTEM_RANGE_CONFIG = 0x09;
    public static final int SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    public static final int SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    public static final int GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    public static final int SYSTEM_INTERRUPT_CLEAR = 0x0B;
    public static final int RESULT_INTERRUPT_STATUS = 0x13;
    public static final int RESULT_RANGE_STATUS = 0x14;
    public static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
    public static final int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
    public static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
    public static final int RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
    public static final int RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
    public static final int ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;
    public static final int I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    public static final int MSRC_CONFIG_CONTROL = 0x60;
    public static final int PRE_RANGE_CONFIG_MIN_SNR = 0x27;
    public static final int PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
    public static final int PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
    public static final int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;
    public static final int FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
    public static final int FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
    public static final int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
    public static final int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    public static final int PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
    public static final int PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
    public static final int PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    public static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    public static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
    public static final int SYSTEM_HISTOGRAM_BIN = 0x81;
    public static final int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
    public static final int HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
    public static final int FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    public static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    public static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
    public static final int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
    public static final int MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    public static final int SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
    public static final int IDENTIFICATION_MODEL_ID = 0xC0;
    public static final int IDENTIFICATION_REVISION_ID = 0xC2;
    public static final int OSC_CALIBRATE_VAL = 0xF8;
    public static final int GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;
    public static final int GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    public static final int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    public static final int DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    public static final int POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
    public static final int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    public static final int ALGO_PHASECAL_LIM = 0x30;
    public static final int ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;

    enum vcselPeriodType {VcselPeriodPreRange, VcselPeriodFinalRange}

    public int measurement_timing_budget_us;
    public byte address;
    public boolean did_timeout;
    public byte stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API


    @Override
    public void init() {
        telemetry.addData("Sensor", "Initialzing");
        lidarSensor = hardwareMap.i2cDevice.get("lidar");
        lidarAddr = I2cAddr.create8bit(LIDAR_ADDRESS);
        lidarSynch = new I2cDeviceSynchImpl(lidarSensor, lidarAddr, false);
        lidarSynch.engage();
    }

    @Override
    public void init_loop() {
        if(!isInitialized) {
            vl53lox_init();
            // Start continuous back-to-back mode (take readings as
            // fast as possible).  To use continuous timed mode
            // instead, provide a desired inter-measurement period in
            // ms (e.g. sensor.startContinuous(100)).
            //startContinuous();
        } else if(lidarSynch.read8(SYSRANGE_START) != 0x02) {
            startContinuous();
        } else {
            telemetry.addData("Sensor", "Initialzed");
            telemetry.addData("Range type", lidarSynch.read8(SYSRANGE_START));
            telemetry.addData("Lidar Model (238)", lidarSynch.read8(IDENTIFICATION_MODEL_ID) + 256);
        }
    }

    @Override
    public void start() {

        timer.reset();
    }

    @Override
    public void loop() {

        sensorStatus = lidarSynch.read8(RESULT_INTERRUPT_STATUS);
        if((sensorStatus & 0x07) == 0) {
            if (timer.milliseconds() > timeout) {
                count++;
                timer.reset();
            } else {
                loopCount++;
            }
        } else if(timer.milliseconds() > 40){
            distance = readRangeContinuousMillimeters();
            readCount++;
            timer.reset();
        }


        // send the info back to driver station using telemetry function.
        telemetry.addData("Timer", timer.toString());
        telemetry.addData("Timeout Count", count);
        telemetry.addData("Read Count", readCount);
        telemetry.addData("Loop Count", loopCount);
        telemetry.addData("Lidar Model (238)", lidarSynch.read8(IDENTIFICATION_MODEL_ID) + 256);
        telemetry.addData("Interrupt", lidarSynch.read8(RESULT_INTERRUPT_STATUS) & 0x07);
        telemetry.addData("Range type", lidarSynch.read8(SYSRANGE_START));
        telemetry.addData("Range in mm:", distance);
        telemetry.addData("Sensor Status", sensorStatus);

    }


    @Override
    public void stop() {
        stopContinuous();
    }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.
// Defines /////////////////////////////////////////////////////////////////////

    // Returns a range reading in millimeters when continuous mode is active
    // (readRangeSingleMillimeters() also calls this function after starting a
    // single-shot range measurement)
    int readRangeContinuousMillimeters() {
        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        byte[] lidarCache = lidarSynch.read(RESULT_RANGE_STATUS + 10, 2);
        //One byte only goes to 256 values.  Two bytes must be combined to get the total distance
        int reading = 0X000000FF & (int) lidarCache[0];
        reading = (reading << 8 | (0X000000FF & (int) lidarCache[1]));

        lidarSynch.write8(SYSTEM_INTERRUPT_CLEAR, 0x01);

        return reading;
    }

    // Start continuous ranging measurements. If period_ms (optional) is 0 or not
    // given, continuous back-to-back mode is used (the sensor takes measurements as
    // often as possible); otherwise, continuous timed mode is used, with the given
    // inter-measurement period in milliseconds determining how often the sensor
    // takes a measurement.
    // based on VL53L0X_StartMeasurement()
    void startContinuous() {
        lidarSynch.write8(0x80, 0x01);
        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x00);
        lidarSynch.write8(0x91, stop_variable);
        lidarSynch.write8(0x00, 0x01);
        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x80, 0x00);

        // continuous back-to-back mode
        lidarSynch.write8(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }

    // Stop continuous measurements
    // based on VL53L0X_StopMeasurement()
    void stopContinuous() {
        lidarSynch.write8(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x00);
        lidarSynch.write8(0x91, 0x00);
        lidarSynch.write8(0x00, 0x01);
        lidarSynch.write8(0xFF, 0x00);
    }


    // Initialize sensor using sequence based on VL53L0X_DataInit(),
    // VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
    // This function does not perform reference SPAD calibration
    // (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
    // is performed by ST on the bare modules; it seems like that should work well
    // enough unless a cover glass is added.
    // If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
    // mode.
    void vl53lox_init() {

        // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode
        lidarSynch.write8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                lidarSynch.read8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0

        // "Set I2C standard mode"
        lidarSynch.write8(0x88, 0x00);

        lidarSynch.write8(0x80, 0x01);
        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x00);
        stop_variable = lidarSynch.read8(0x91);
        lidarSynch.write8(0x00, 0x01);
        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x80, 0x00);

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        lidarSynch.write8(MSRC_CONFIG_CONTROL, lidarSynch.read8(MSRC_CONFIG_CONTROL) | 0x12);

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        //setSignalRateLimit(0.25);

        lidarSynch.write8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

        // VL53L0X_DataInit() end//////////////////////////////////////////////////////////////////////
/*
        // VL53L0X_StaticInit() begin//////////////////////////////////////////////////////////////////

        byte spad_count;
        boolean spad_type_is_aperture;
        if (!getSpadInfo( & spad_count, &spad_type_is_aperture)){
            spad_type_is_aperture = false;
        }

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        byte[] ref_spad_map;
        ref_spad_map = lidarSynch.read(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6);

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        lidarSynch.write8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        int first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
        byte spads_enabled = 0;

        for (int i = 0; i < 48; i++) {
            if (i < first_spad_to_enable || spads_enabled == spad_count) {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
                spads_enabled++;
            }
        }

        lidarSynch.write(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

        // -- VL53L0X_set_reference_spads() end
*/
        // -- VL53L0X_load_tuning_settings() begin//////////////////////////////////////////////////
        // DefaultTuningSettings from vl53l0x_tuning.h

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x00);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x09, 0x00);
        lidarSynch.write8(0x10, 0x00);
        lidarSynch.write8(0x11, 0x00);

        lidarSynch.write8(0x24, 0x01);
        lidarSynch.write8(0x25, 0xFF);
        lidarSynch.write8(0x75, 0x00);

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x4E, 0x2C);
        lidarSynch.write8(0x48, 0x00);
        lidarSynch.write8(0x30, 0x20);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x30, 0x09);
        lidarSynch.write8(0x54, 0x00);
        lidarSynch.write8(0x31, 0x04);
        lidarSynch.write8(0x32, 0x03);
        lidarSynch.write8(0x40, 0x83);
        lidarSynch.write8(0x46, 0x25);
        lidarSynch.write8(0x60, 0x00);
        lidarSynch.write8(0x27, 0x00);
        lidarSynch.write8(0x50, 0x06);
        lidarSynch.write8(0x51, 0x00);
        lidarSynch.write8(0x52, 0x96);
        lidarSynch.write8(0x56, 0x08);
        lidarSynch.write8(0x57, 0x30);
        lidarSynch.write8(0x61, 0x00);
        lidarSynch.write8(0x62, 0x00);
        lidarSynch.write8(0x64, 0x00);
        lidarSynch.write8(0x65, 0x00);
        lidarSynch.write8(0x66, 0xA0);

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x22, 0x32);
        lidarSynch.write8(0x47, 0x14);
        lidarSynch.write8(0x49, 0xFF);
        lidarSynch.write8(0x4A, 0x00);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x7A, 0x0A);
        lidarSynch.write8(0x7B, 0x00);
        lidarSynch.write8(0x78, 0x21);

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x23, 0x34);
        lidarSynch.write8(0x42, 0x00);
        lidarSynch.write8(0x44, 0xFF);
        lidarSynch.write8(0x45, 0x26);
        lidarSynch.write8(0x46, 0x05);
        lidarSynch.write8(0x40, 0x40);
        lidarSynch.write8(0x0E, 0x06);
        lidarSynch.write8(0x20, 0x1A);
        lidarSynch.write8(0x43, 0x40);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x34, 0x03);
        lidarSynch.write8(0x35, 0x44);

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x31, 0x04);
        lidarSynch.write8(0x4B, 0x09);
        lidarSynch.write8(0x4C, 0x05);
        lidarSynch.write8(0x4D, 0x04);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x44, 0x00);
        lidarSynch.write8(0x45, 0x20);
        lidarSynch.write8(0x47, 0x08);
        lidarSynch.write8(0x48, 0x28);
        lidarSynch.write8(0x67, 0x00);
        lidarSynch.write8(0x70, 0x04);
        lidarSynch.write8(0x71, 0x01);
        lidarSynch.write8(0x72, 0xFE);
        lidarSynch.write8(0x76, 0x00);
        lidarSynch.write8(0x77, 0x00);

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x0D, 0x01);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x80, 0x01);
        lidarSynch.write8(0x01, 0xF8);

        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x8E, 0x01);
        lidarSynch.write8(0x00, 0x01);
        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x80, 0x00);

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        lidarSynch.write8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        lidarSynch.write8(GPIO_HV_MUX_ACTIVE_HIGH, lidarSynch.read8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
        lidarSynch.write8(SYSTEM_INTERRUPT_CLEAR, 0x01);

        // -- VL53L0X_SetGpioConfig() end

        //measurement_timing_budget_us = getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        lidarSynch.write8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        //setMeasurementTimingBudget(measurement_timing_budget_us);

        // VL53L0X_StaticInit() end///////////////////////////////////////////////////////////////////////////////////////////

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        lidarSynch.write8(SYSTEM_SEQUENCE_CONFIG, 0x01);
        /*if (!performSingleRefCalibration(0x40)) {
            return false;
        }
*/
        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        /*lidarSynch.write8(SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!performSingleRefCalibration(0x00)) {
            return false;
        }
*/
        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        lidarSynch.write8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // VL53L0X_PerformRefCalibration() end//////////////////////////////////////////////////////////////////

        isInitialized = true;
    }

/*
    // Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
    boolean setMeasurementTimingBudget(int budget_us) {
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        final int StartOverhead = 1320; // note that this is different than the value in get_
        final int EndOverhead = 960;
        final int MsrcOverhead = 660;
        final int TccOverhead = 590;
        final int DssOverhead = 690;
        final int PreRangeOverhead = 660;
        final int FinalRangeOverhead = 550;

        final int MinTimingBudget = 20000;

        if (budget_us < MinTimingBudget) {
            return false;
        }

        int used_budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables( & enables);
        getSequenceStepTimeouts( & enables, &timeouts);

        if (enables.tcc) {
            used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss) {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        } else if (enables.msrc) {
            used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range) {
            used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range) {
            used_budget_us += FinalRangeOverhead;

            // "Note that the final range timeout is determined by the timing
            // budget and the sum of all other timeouts within the sequence.
            // If there is no room for the final range timeout, then an error
            // will be set. Otherwise the remaining time will be applied to
            // the final range."

            if (used_budget_us > budget_us) {
                // "Requested timeout too big."
                return false;
            }

            int final_range_timeout_us = budget_us - used_budget_us;

            // set_sequence_step_timeout() begin
            // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

            // "For the final range timeout, the pre-range timeout
            //  must be added. To do this both final and pre-range
            //  timeouts must be expressed in macro periods MClks
            //  because they have different vcsel periods."

            int final_range_timeout_mclks =
                    timeoutMicrosecondsToMclks(final_range_timeout_us,
                            timeouts.final_range_vcsel_period_pclks);

            if (enables.pre_range) {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }

            lidarSynch.write(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encodeTimeout(final_range_timeout_mclks));

            // set_sequence_step_timeout() end

            measurement_timing_budget_us = budget_us; // store for internal reuse
        }
        return true;
    }
*/

/*
    // Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
    int getMeasurementTimingBudget(void) {
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        final int StartOverhead = 1910; // note that this is different than the value in set_
        final int EndOverhead = 960;
        final int MsrcOverhead = 660;
        final int TccOverhead = 590;
        final int DssOverhead = 690;
        final int PreRangeOverhead = 660;
        final int FinalRangeOverhead = 550;

        // "Start and end overhead times always present"
        int budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables( & enables);
        getSequenceStepTimeouts( & enables, &timeouts);

        if (enables.tcc) {
            budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss) {
            budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        } else if (enables.msrc) {
            budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range) {
            budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range) {
            budget_us += (timeouts.final_range_us + FinalRangeOverhead);
        }

        measurement_timing_budget_us = budget_us; // store for internal reuse
        return budget_us;
    }
*/

/*
// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
        bool VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
        {
        uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        getSequenceStepEnables(&enables);
        getSequenceStepTimeouts(&enables, &timeouts);

        // "Apply specific settings for the requested clock period"
        // "Re-calculate and apply timeouts, in macro periods"

        // "When the VCSEL period for the pre or final range is changed,
        // the corresponding timeout must be read from the device using
        // the current VCSEL period, then the new VCSEL period can be
        // applied. The timeout then must be written back to the device
        // using the new VCSEL period.
        //
        // For the MSRC timeout, the same applies - this timeout being
        // dependant on the pre-range vcsel period."


        if (type == VcselPeriodPreRange)
        {
        // "Set phase check limits"
        switch (period_pclks)
        {
        case 12:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

        case 14:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

        case 16:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

        case 18:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

default:
        // invalid period
        return false;
        }
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks =
        timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks =
        timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
        (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
        }
        else if (type == VcselPeriodFinalRange)
        {
        switch (period_pclks)
        {
        case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x30);
        writeReg(0xFF, 0x00);
        break;

        case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

        case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

        case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

default:
        // invalid period
        return false;
        }

        // apply new VCSEL period
        writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks =
        timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range)
        {
        new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(new_final_range_timeout_mclks));

        // set_sequence_step_timeout end
        }
        else
        {
        // invalid type
        return false;
        }

        // "Finally, the timing budget must be re-applied"

        setMeasurementTimingBudget(measurement_timing_budget_us);

        // "Perform the phase calibration. This is needed after changing on vcsel period."
        // VL53L0X_perform_phase_calibration() begin

        uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
        performSingleRefCalibration(0x0);
        writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

        // VL53L0X_perform_phase_calibration() end

        return true;
        }
*/

/*
// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
        uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type)
        {
        if (type == VcselPeriodPreRange)
        {
        return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else if (type == VcselPeriodFinalRange)
        {
        return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else { return 255; }
        }
*/






/*
    // Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
    int readRangeSingleMillimeters() {
        lidarSynch.write8(0x80, 0x01);
        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x00);
        lidarSynch.write8(0x91, stop_variable);
        lidarSynch.write8(0x00, 0x01);
        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x80, 0x00);

        lidarSynch.write8(SYSRANGE_START, 0x01);

        // "Wait until start bit has been cleared"
        //startTimeout();
        while (lidarSynch.read8(SYSRANGE_START) & 0x01) {
            if (checkTimeoutExpired()) {
                did_timeout = true;
                return 65535;
            }
        }

        return readRangeContinuousMillimeters();
    }
*/
    // Private Methods /////////////////////////////////////////////////////////////

/*
    // Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
    boolean getSpadInfo(int count, boolean type_is_aperture) {
        byte tmp;

        lidarSynch.write8(0x80, 0x01);
        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x00);

        lidarSynch.write8(0xFF, 0x06);
        lidarSynch.write8(0x83, lidarSynch.read8(0x83) | 0x04);
        lidarSynch.write8(0xFF, 0x07);
        lidarSynch.write8(0x81, 0x01);

        lidarSynch.write8(0x80, 0x01);

        lidarSynch.write8(0x94, 0x6b);
        lidarSynch.write8(0x83, 0x00);
        //startTimeout();
        while (lidarSynch.read8(0x83) == 0x00) {
            if (checkTimeoutExpired()) {
                return false;
            }
        }
        lidarSynch.write8(0x83, 0x01);

        tmp = lidarSynch.read8(0x92);

        count = tmp & 0x7f;
        type_is_aperture = (tmp >> 7) & 0x01;

        lidarSynch.write8(0x81, 0x00);
        lidarSynch.write8(0xFF, 0x06);
        lidarSynch.write8(0x83, lidarSynch.read8(0x83) & ~0x04);
        lidarSynch.write8(0xFF, 0x01);
        lidarSynch.write8(0x00, 0x01);

        lidarSynch.write8(0xFF, 0x00);
        lidarSynch.write8(0x80, 0x00);

        return true;
    }
*/

 /*   // Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
    void getSequenceStepEnables(SequenceStepEnables *enables) {
        byte sequence_config = lidarSynch.read8(SYSTEM_SEQUENCE_CONFIG);

        enables -> tcc = (sequence_config >> 4) & 0x1;
        enables -> dss = (sequence_config >> 3) & 0x1;
        enables -> msrc = (sequence_config >> 2) & 0x1;
        enables -> pre_range = (sequence_config >> 6) & 0x1;
        enables -> final_range = (sequence_config >> 7) & 0x1;
    }
*/
/*
    // Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
    void getSequenceStepTimeouts(SequenceStepEnables const*enables, SequenceStepTimeouts *timeouts) {
        timeouts -> pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

        timeouts -> msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        timeouts -> msrc_dss_tcc_us =
                timeoutMclksToMicroseconds(timeouts -> msrc_dss_tcc_mclks,
                        timeouts -> pre_range_vcsel_period_pclks);

        timeouts -> pre_range_mclks =
                decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        timeouts -> pre_range_us =
                timeoutMclksToMicroseconds(timeouts -> pre_range_mclks,
                        timeouts -> pre_range_vcsel_period_pclks);

        timeouts -> final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

        timeouts -> final_range_mclks =
                decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

        if (enables -> pre_range) {
            timeouts -> final_range_mclks -= timeouts -> pre_range_mclks;
        }

        timeouts -> final_range_us =
                timeoutMclksToMicroseconds(timeouts -> final_range_mclks,
                        timeouts -> final_range_vcsel_period_pclks);
    }
*/

 /*   // Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
    uint16_t VL53L0X::

    decodeTimeout(uint16_t reg_val) {
        // format: "(LSByte * 2^MSByte) + 1"
        return (uint16_t) ((reg_val & 0x00FF) <<
                (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
    }

    // Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
    uint16_t VL53L0X::

    encodeTimeout(uint16_t timeout_mclks) {
        // format: "(LSByte * 2^MSByte) + 1"

        uint32_t ls_byte = 0;
        uint16_t ms_byte = 0;

        if (timeout_mclks > 0) {
            ls_byte = timeout_mclks - 1;

            while ((ls_byte & 0xFFFFFF00) > 0) {
                ls_byte >>= 1;
                ms_byte++;
            }

            return (ms_byte << 8) | (ls_byte & 0xFF);
        } else {
            return 0;
        }
    }
*/
/*
    // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
    uint32_t VL53L0X::

    timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
        uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    // Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
    uint32_t VL53L0X::

    timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
        uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    }
*/


 /*   // based on VL53L0X_perform_single_ref_calibration()
    boolean performSingleRefCalibration(byte vhv_init_byte) {
        lidarSynch.write8(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

        //startTimeout();
        while ((lidarSynch.read8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            if (checkTimeoutExpired()) {
                return false;
            }
        }

        lidarSynch.write8(SYSTEM_INTERRUPT_CLEAR, 0x01);

        lidarSynch.write8(SYSRANGE_START, 0x00);

        return true;
    }
*/
}
