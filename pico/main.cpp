#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdio.h"

enum class GainFactor
{
    k1x = 0,
    k3x = 1,
    k6x = 2,
    k9x = 3,
    k18x = 4
};

enum class LEDPulseFrequency
{
    k60kHz = 0x18,
    k70kHz = 0x40,
    k80kHz = 0x28,
    k90kHz = 0x30,
    k100kHz = 0x38,
};

enum class LEDCurrent
{
    kPulse2mA = 0,
    kPulse5mA = 1,
    kPulse10mA = 2,
    kPulse25mA = 3,
    kPulse50mA = 4,
    kPulse75mA = 5,
    kPulse100mA = 6,
    kPulse125mA = 7,
};

enum class ProximityResolution
{
    k8bit = 0x00,
    k9bit = 0x08,
    k10bit = 0x10,
    k11bit = 0x18,
};

enum class ProximityMeasurementRate
{
    k6ms = 1,
    k12ms = 2,
    k25ms = 3,
    k50ms = 4,
    k100ms = 5,
    k200ms = 6,
    k400ms = 7,
};

enum class ColorResolution
{
    k20bit = 0x00,
    k19bit = 0x10,
    k18bit = 0x20,
    k17bit = 0x30,
    k16bit = 0x40,
    k13bit = 0x50,
};

enum class ColorMeasurementRate
{
    k25ms = 0,
    k50ms = 1,
    k100ms = 2,
    k200ms = 3,
    k500ms = 4,
    k1000ms = 5,
    k2000ms = 7,
};

enum class Register
{
    kMainCtrl = 0x00,
    kProximitySensorLED = 0x01,
    kProximitySensorPulses = 0x02,
    kProximitySensorRate = 0x03,
    kLightSensorMeasurementRate = 0x04,
    kLightSensorGain = 0x05,
    kPartID = 0x06,
    kMainStatus = 0x07,
    kProximityData = 0x08,
    kDataInfrared = 0x0A,
    kDataGreen = 0x0D,
    kDataBlue = 0x10,
    kDataRed = 0x13
};

enum class MainCtrlFields
{
    kProximitySensorEnable = 0x01,
    kLightSensorEnable = 0x02,
    kRGBMode = 0x04
};

static constexpr int kAddress = 0x52;
static constexpr int kExpectedPartID = 0xC2;
static constexpr int kMuxAddress = 0x70;
static constexpr int kSensorCount = 3;
// static constexpr GainFactor kDefaultGain =
//     GainFactor::k3x;
// static constexpr LEDPulseFrequency kDefaultPulseFreq =
//     LEDPulseFrequency::k60kHz;
// static constexpr LEDCurrent kDefaultLEDCurrent =
//     LEDCurrent::kPulse100mA;
// static constexpr uint8_t kDefaultPulses = 32;
// static constexpr ProximityResolution kDefaultProxRes =
//     ProximityResolution::k11bit;
// static constexpr ProximityMeasurementRate kDefaultProxRate =
//     ProximityMeasurementRate::k50ms;
// static constexpr ColorResolution kDefaultColorRes =
//     ColorResolution::k18bit;
// static constexpr ColorMeasurementRate kDefaultColorRate =
//     ColorMeasurementRate::k100ms;

bool read_i2c_data(i2c_inst_t *i2c, Register reg, uint8_t *buffer, uint8_t readLen)
{
    buffer[0] = static_cast<uint8_t>(reg);
    int result = i2c_write_timeout_us(i2c, kAddress, buffer, 1, true, 25000);
    if (result != 1)
    {
        return false;
    }
    result = i2c_read_timeout_us(i2c, kAddress, buffer, readLen, false, 25000);
    return result == readLen;
}

bool select_channel(i2c_inst_t *i2c, uint8_t *buffer, uint8_t ch) {
    int result;
    buffer[0] = 1 << ch;
    result = i2c_write_timeout_us(i2c, kMuxAddress, buffer, 1, false, 25000);
    return ( result == 1 );
}

void init_device(i2c_inst_t *i2c, uint8_t *i2cBuffer)
{
    i2cBuffer[0] = static_cast<uint8_t>(Register::kMainCtrl);
    i2cBuffer[1] = static_cast<uint8_t>(MainCtrlFields::kRGBMode) |
                   static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) |
                   static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable);
    i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorRate);
    i2cBuffer[1] = static_cast<uint8_t>(ProximityResolution::k11bit) | static_cast<uint8_t>(ProximityMeasurementRate::k100ms);
    i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorPulses);
    i2cBuffer[1] = 32;
    i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    // i2cBuffer[0] = static_cast<uint8_t>(Register::kLightSensorMeasurementRate);
    // i2cBuffer[1] = static_cast<uint8_t>(kDefaultColorRes) | static_cast<uint8_t>(kDefaultColorRate);
    // i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    // i2cBuffer[0] = static_cast<uint8_t>(Register::kLightSensorGain);
    // i2cBuffer[1] = static_cast<uint8_t>(kDefaultGain);
    // i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);
}

void blink_led() {
    // Blink an onboard led in order to provide some visual debugging
#ifndef PICO_DEFAULT_LED_PIN
#warning Not able to compile led blink
#else
    static bool init = true;
    static bool state = true;
    if ( init ) {
       gpio_init(PICO_DEFAULT_LED_PIN);
       gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
       init = false;
    };

    if ( state ){
       gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }else{
       gpio_put(PICO_DEFAULT_LED_PIN, 0);
    };
    state = !state;
#endif
};

int main()
{
	
	
    // Allow stdio to be sent to the uart in order to easily switch between usb serial and uart for testing.
    stdio_uart_init_full (uart0, 115200, PICO_DEFAULT_UART_TX_PIN, PICO_DEFAULT_UART_RX_PIN);
    stdio_usb_init();
    stdio_init_all();

    blink_led(); // turn on LED for half a second.
    sleep_ms(1000);
    if ( stdio_usb_connected() ) {
		for ( int i=0; i<3; i++){
		  blink_led();
		  sleep_ms(500);
	    };
    };

    // Init I2c 0
    i2c_init(i2c0, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Channel 0 output
    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);
    gpio_put(14, 1);

    char outputBuffer[512];

    unsigned int values[kSensorCount*5];
    uint8_t i2cBuffer[20];
    bool sensorValid[kSensorCount];
    absolute_time_t loopTime;
    bool reset;

    sleep_ms(100);

    for ( int i=0; i<kSensorCount; i++ ){
      sensorValid[i]=false;
      select_channel(i2c0, i2cBuffer, i);
      init_device(i2c0, i2cBuffer);
      read_i2c_data(i2c0, Register::kMainStatus, i2cBuffer, 1);
    }

    memset(values, 0, sizeof(values));

    while (1)
    {
	blink_led();
        loopTime = make_timeout_time_ms(100);

	for ( int i=0; i<kSensorCount; i++){
	  select_channel(i2c0, i2cBuffer, i);
          sensorValid[i] = read_i2c_data(i2c0, Register::kMainStatus, i2cBuffer, 15);
          if (sensorValid[i])
          {
              if ((i2cBuffer[0] & 0x20) != 0)
              {
                  init_device(i2c0, i2cBuffer);
              }
              else
              {
		  int base=i*5;
                  values[base+4] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
                  values[base+3] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
                  values[base+2] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
                  values[base+1] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
                  values[base] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
              }
          }
        };

	for ( int i=0; i<kSensorCount; i++){
	  printf("%u,",sensorValid[i]);
	};
	for ( int i=0; i<(kSensorCount*5)-1; i++){
	  printf("%u,",values[i]);
	};
	printf("%u\n",values[(kSensorCount*5)-1]);

        //snprintf(outputBuffer, sizeof(outputBuffer), "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                // currentValid0, currentValid1, values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9]);

        //uart_puts(uart0, outputBuffer);

        sleep_until(loopTime);
    }
}
