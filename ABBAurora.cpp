#include "ABBAurora.h"
#include "ABBAuroraStrings.h"
#include "esphome/core/log.h"

static const char *const TAG = "ABB_AURORA";

byte ABBAurora::TXPinControl;
byte ABBAurora::RXPin;
byte ABBAurora::TXPin;
HardwareSerial *ABBAurora::serial;

ABBAurora::ABBAurora(byte address)
{
    Address = address;
    SendStatus = false;
    ReceiveStatus = false;
    clearReceiveData();
}
void ABBAurora::setup(HardwareSerial &hardwareSerial, byte RXGpioPin, byte TXGpioPin, byte TXControllPin)
{
    TXPinControl = TXControllPin;
    RXPin = RXGpioPin;
    TXPin = TXGpioPin;

    pinMode(TXPinControl, OUTPUT);
    digitalWrite(TXPinControl, LOW);

    serial = &hardwareSerial;
#if defined(USE_ESP32)
    serial->begin(19200, SERIAL_8N1, RXGpioPin, TXGpioPin, false, 500);
#elif defined(USE_ESP8266)
    serial->begin(19200, SERIAL_8N1, SERIAL_FULL);
    if (RXGpioPin == 13 && TXGpioPin == 15) {
        // Use alternate pins on Serial
        serial->swap();
    }
#endif
    // Shorten readBytes() timeout from Arduino's 1000ms default so the main
    // loop isn't stalled for tens of seconds per poll cycle by the many
    // unanswered/unsupported DSP queries (each would otherwise cost a full
    // second). The occasional short, garbage-looking partial reads seen at
    // both 100ms and 250ms look like RS-485 line noise, not a timing issue.
    serial->setTimeout(100);
}

void ABBAurora::reset_serial()
{
    ESP_LOGW(TAG, "Resetting UART after a run of consecutive read failures (possible driver lockup)");
    serial->end();
    delay(100);
#if defined(USE_ESP32)
    serial->begin(19200, SERIAL_8N1, RXPin, TXPin, false, 500);
#elif defined(USE_ESP8266)
    serial->begin(19200, SERIAL_8N1, SERIAL_FULL);
    if (RXPin == 13 && TXPin == 15) {
        serial->swap();
    }
#endif
    serial->setTimeout(100);
}

void ABBAurora::clearData(byte *data, byte len)
{
    for (int i = 0; i < len; i++)
    {
        data[i] = 0;
    }
}

int ABBAurora::Crc16(byte *data, int offset, int count)
{
    byte BccLo = 0xFF;
    byte BccHi = 0xFF;

    for (int i = offset; i < (offset + count); i++)
    {
        byte New = data[offset + i] ^ BccLo;
        byte Tmp = New << 4;
        New = Tmp ^ New;
        Tmp = New >> 5;
        BccLo = BccHi;
        BccHi = New ^ Tmp;
        Tmp = New << 3;
        BccLo = BccLo ^ Tmp;
        Tmp = New >> 4;
        BccLo = BccLo ^ Tmp;
    }

    return (int)word(~BccHi, ~BccLo);
}

bool ABBAurora::Send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, byte param5, byte param6)
{

    SendStatus = false;
    ReceiveStatus = false;

    byte SendData[10];
    SendData[0] = address;
    SendData[1] = param0;
    SendData[2] = param1;
    SendData[3] = param2;
    SendData[4] = param3;
    SendData[5] = param4;
    SendData[6] = param5;
    SendData[7] = param6;

    int crc = Crc16(SendData, 0, 8);
    SendData[8] = lowByte(crc);
    SendData[9] = highByte(crc);

    ESP_LOGD(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             SendData[0], SendData[1], SendData[2], SendData[3], SendData[4],
             SendData[5], SendData[6], SendData[7], SendData[8], SendData[9]);

    clearReceiveData();

    for (int i = 0; i < this->MaxAttempt; i++)
    {
        digitalWrite(TXPinControl, RS485Transmit);
        delay(40);

        size_t written = serial->write(SendData, sizeof(SendData));
        ESP_LOGD(TAG, "Wrote %u/%u bytes to serial", (unsigned)written, (unsigned)sizeof(SendData));

        if (written != 0)
        {
            serial->flush();
            SendStatus = true;

            digitalWrite(TXPinControl, RS485Receive);

            size_t got = serial->readBytes(ReceiveData, sizeof(ReceiveData));
            ESP_LOGD(TAG, "RX: got %u/%u bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                     (unsigned)got, (unsigned)sizeof(ReceiveData),
                     ReceiveData[0], ReceiveData[1], ReceiveData[2], ReceiveData[3],
                     ReceiveData[4], ReceiveData[5], ReceiveData[6], ReceiveData[7]);

            if (got != 0)
            {
                int expected_crc = Crc16(ReceiveData, 0, 6);
                int actual_crc = (int)word(ReceiveData[7], ReceiveData[6]);
                ESP_LOGD(TAG, "CRC: computed=0x%04X received=0x%04X (%s)", expected_crc, actual_crc,
                         expected_crc == actual_crc ? "MATCH" : "MISMATCH");

                if (actual_crc == expected_crc)
                {
                    ReceiveStatus = true;
                    break;
                }
            }
        }
    }

    ESP_LOGD(TAG, "Send() attempt(s) done, returning %s", ReceiveStatus ? "true" : "false");
    return ReceiveStatus;
}

void ABBAurora::clearReceiveData()
{
    clearData(ReceiveData, 8);
}

/**
 * Reads a single value of the digital signal procesor.
 * Not all values are supported by all models. 
 * Read values are in following Units:
 * Voltage V
 * Current A 
 * Power W 
 * Temperature °C 
 * 
 **/
bool ABBAurora::ReadDSPValue(DSP_VALUE_TYPE type, DSP_GLOBAL global)
{
    if ((((int)type >= 1 && (int)type <= 9) || ((int)type >= 21 && (int)type <= 63)) && ((int)global >= 0 && (int)global <= 1))
    {
        DSP.ReadState = Send(this->Address, (byte)59, type, global, (byte)0, (byte)0, (byte)0, (byte)0);

        if (DSP.ReadState == false)
        {
            ReceiveData[0] = 255;
            ReceiveData[1] = 255;
        }
    }
    else
    {
        DSP.ReadState = false;
        clearReceiveData();
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
    }

    DSP.TransmissionState = ReceiveData[0];
    DSP.GlobalState = ReceiveData[1];

    foo.asBytes[0] = ReceiveData[5];
    foo.asBytes[1] = ReceiveData[4];
    foo.asBytes[2] = ReceiveData[3];
    foo.asBytes[3] = ReceiveData[2];

    DSP.Value = foo.asFloat;

    return DSP.ReadState;
}

bool ABBAurora::ReadTimeDate()
{
    TimeDate.ReadState = Send(this->Address, (byte)70, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (TimeDate.ReadState == false)
    {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
    }

    TimeDate.TransmissionState = ReceiveData[0];
    TimeDate.GlobalState = ReceiveData[1];
    TimeDate.Seconds = ((unsigned long)ReceiveData[2] << 24) + ((unsigned long)ReceiveData[3] << 16) + ((unsigned long)ReceiveData[4] << 8) + (unsigned long)ReceiveData[5];
    return TimeDate.ReadState;
}

bool ABBAurora::ReadLastFourAlarms()
{
    LastFourAlarms.ReadState = Send(this->Address, (byte)86, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (LastFourAlarms.ReadState == false)
    {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
        ReceiveData[2] = 255;
        ReceiveData[3] = 255;
        ReceiveData[4] = 255;
        ReceiveData[5] = 255;
    }

    LastFourAlarms.TransmissionState = ReceiveData[0];
    LastFourAlarms.GlobalState = ReceiveData[1];
    LastFourAlarms.Alarms1 = ReceiveData[2];
    LastFourAlarms.Alarms2 = ReceiveData[3];
    LastFourAlarms.Alarms3 = ReceiveData[4];
    LastFourAlarms.Alarms4 = ReceiveData[5];

    return LastFourAlarms.ReadState;
}

bool ABBAurora::ReadJunctionBoxState(byte nj)
{
    return Send(this->Address, (byte)200, nj, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadJunctionBoxVal(byte nj, byte par)
{
    return Send(this->Address, (byte)201, nj, par, (byte)0, (byte)0, (byte)0, (byte)0);
}

// Inverters

bool ABBAurora::ReadSystemPN()
{
    SystemPN.ReadState = Send(this->Address, (byte)52, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    SystemPN.PN = String(String((char)ReceiveData[0]) + String((char)ReceiveData[1]) + String((char)ReceiveData[2]) + String((char)ReceiveData[3]) + String((char)ReceiveData[4]) + String((char)ReceiveData[5]));

    return SystemPN.ReadState;
}

bool ABBAurora::ReadSystemSerialNumber()
{
    SystemSerialNumber.ReadState = Send(this->Address, (byte)63, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    SystemSerialNumber.SerialNumber = String(String((char)ReceiveData[0]) + String((char)ReceiveData[1]) + String((char)ReceiveData[2]) + String((char)ReceiveData[3]) + String((char)ReceiveData[4]) + String((char)ReceiveData[5]));

    return SystemSerialNumber.ReadState;
}

bool ABBAurora::ReadManufacturingWeekYear()
{
    ManufacturingWeekYear.ReadState = Send(this->Address, (byte)65, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (ManufacturingWeekYear.ReadState == false)
    {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
    }

    ManufacturingWeekYear.TransmissionState = ReceiveData[0];
    ManufacturingWeekYear.GlobalState = ReceiveData[1];
    ManufacturingWeekYear.Week = String(String((char)ReceiveData[2]) + String((char)ReceiveData[3]));
    ManufacturingWeekYear.Year = String(String((char)ReceiveData[4]) + String((char)ReceiveData[5]));

    return ManufacturingWeekYear.ReadState;
}

bool ABBAurora::ReadFirmwareRelease()
{
    FirmwareRelease.ReadState = Send(this->Address, (byte)72, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (FirmwareRelease.ReadState == false)
    {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
    }

    FirmwareRelease.TransmissionState = ReceiveData[0];
    FirmwareRelease.GlobalState = ReceiveData[1];
    FirmwareRelease.Release = String(String((char)ReceiveData[2]) + "." + String((char)ReceiveData[3]) + "." + String((char)ReceiveData[4]) + "." + String((char)ReceiveData[5]));

    return FirmwareRelease.ReadState;
}

bool ABBAurora::ReadCumulatedEnergy(CUMULATED_ENERGY_TYPE par)
{
    if ((int)par >= 0 && (int)par <= 6)
    {
        CumulatedEnergy.ReadState = Send(this->Address, (byte)78, par, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

        if (CumulatedEnergy.ReadState == false)
        {
            ReceiveData[0] = 255;
            ReceiveData[1] = 255;
        }
    }
    else
    {
        CumulatedEnergy.ReadState = false;
        clearReceiveData();
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
    }

    CumulatedEnergy.TransmissionState = ReceiveData[0];
    CumulatedEnergy.GlobalState = ReceiveData[1];
    if (CumulatedEnergy.ReadState == true)
    {
        ulo.asBytes[0] = ReceiveData[5];
        ulo.asBytes[1] = ReceiveData[4];
        ulo.asBytes[2] = ReceiveData[3];
        ulo.asBytes[3] = ReceiveData[2];

        CumulatedEnergy.Energy = ulo.asUlong;
    }
    return CumulatedEnergy.ReadState;
}

bool ABBAurora::WriteBaudRateSetting(byte baudcode)
{
    if ((int)baudcode >= 0 && (int)baudcode <= 3)
    {
        return Send(this->Address, (byte)85, baudcode, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    }
    else
    {
        clearReceiveData();
        return false;
    }
}

// Central
bool ABBAurora::ReadFlagsSwitchCentral()
{
    return Send(this->Address, (byte)67, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadCumulatedEnergyCentral(byte var, byte ndays_h, byte ndays_l, byte global)
{
    return Send(this->Address, (byte)68, var, ndays_h, ndays_l, global, (byte)0, (byte)0);
}

bool ABBAurora::ReadFirmwareReleaseCentral(byte var)
{
    return Send(this->Address, (byte)72, var, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadBaudRateSettingCentral(byte baudcode, byte serialline)
{
    return Send(this->Address, (byte)85, baudcode, serialline, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadSystemInfoCentral(byte var)
{
    return Send(this->Address, (byte)101, var, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadJunctionBoxMonitoringCentral(byte cf, byte rn, byte njt, byte jal, byte jah)
{
    return Send(this->Address, (byte)103, cf, rn, njt, jal, jah, (byte)0);
}

bool ABBAurora::ReadSystemPNCentral()
{
    return Send(this->Address, (byte)105, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadSystemSerialNumberCentral()
{
    return Send(this->Address, (byte)107, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
}

bool ABBAurora::ReadState()
{
    State.ReadState = Send(this->Address, (byte)50, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (State.ReadState == false)
    {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
        ReceiveData[2] = 255;
        ReceiveData[3] = 255;
        ReceiveData[4] = 255;
        ReceiveData[5] = 255;
    }

    State.TransmissionState = ReceiveData[0];
    State.GlobalState = ReceiveData[1];
    State.InverterState = ReceiveData[2];
    State.Channel1State = ReceiveData[3];
    State.Channel2State = ReceiveData[4];
    State.AlarmState = ReceiveData[5];

    return State.ReadState;
}

bool ABBAurora::ReadVersion()
{
    Version.ReadState = Send(this->Address, (byte)58, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (Version.ReadState == false)
    {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
    }

    Version.TransmissionState = ReceiveData[0];
    Version.GlobalState = ReceiveData[1];

    Version.Par1 = ABBAuroraStrings::VersionPart1(ReceiveData[2]);
    Version.Par2 = ABBAuroraStrings::VersionPart2(ReceiveData[3]);
    Version.Par3 = ABBAuroraStrings::VersionPart3(ReceiveData[4]);
    Version.Par4 = ABBAuroraStrings::VersionPart4(ReceiveData[5]);

    return Version.ReadState;
}
