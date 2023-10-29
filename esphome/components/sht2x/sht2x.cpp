#include "sht2x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sht2x {

static const char *const TAG = "sht2x";

static const uint16_t SHT2X_COMMAND_READ_SERIAL_NUMBER = 0x3780;
static const uint16_t SHT2X_COMMAND_READ_STATUS = 0xF32D;
static const uint16_t SHT2X_COMMAND_CLEAR_STATUS = 0x3041;
static const uint16_t SHT2X_COMMAND_HEATER_ENABLE = 0x306D;
static const uint16_t SHT2X_COMMAND_HEATER_DISABLE = 0x3066;
static const uint8_t SHT2X_COMMAND_SOFT_RESET = 0xFE;
static const uint8_t SHT2X_COMMAND_HUMIDITY = 0xF5;
static const uint16_t SHT2X_COMMAND_POLLING_H = 0x2400;
static const uint16_t SHT2X_COMMAND_FETCH_DATA = 0xE000;

uint8_t SHT2XComponent::crc8(const uint8_t *data, uint8_t len)
{
  //  CRC-8 formula from page 14 of SHT spec pdf
  //  Sensirion_Humidity_Sensors_SHT2x_CRC_Calculation.pdf
  const uint8_t POLY = 0x31;
  uint8_t crc = 0x00;

  for (uint8_t j = len; j; --j)
  {
    crc ^= *data++;

    for (uint8_t i = 8; i; --i)
    {
      crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
    }
  }
  return crc;
}

void SHT2XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up sht2x...");
  if (!this->write_command(SHT2X_COMMAND_SOFT_RESET)) {
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "SHT2X soft reset done.");
}

void SHT2XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "sht2x:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with SHT2X failed!");
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

float SHT2XComponent::get_setup_priority() const { return setup_priority::DATA; }

void SHT2XComponent::update() {
  ESP_LOGD(TAG, "Updating SHT2X...");
  if (this->status_has_warning()) {
    ESP_LOGD(TAG, "Retrying to reconnect the sensor.");
    this->write_command(SHT2X_COMMAND_SOFT_RESET);
  }

  // read humidity
  ESP_LOGD(TAG, "Reading humidity...");
  this->write(&SHT2X_COMMAND_HUMIDITY, 1);
  ESP_LOGD(TAG, "Reading humidity done.");

  delay(100);
  uint8_t raw_humidity[3];
  if (!this->read(raw_humidity, 3)) {
    this->status_set_warning();
    return;
  }
  ESP_LOGD(TAG, "Got hum=%.2f", raw_humidity);


  // this->set_timeout(50, [this]() {
  //   uint16_t raw_humidity[2];
  //   if (!this->read_data(raw_humidity, 2)) {
  //     this->status_set_warning();
  //     return;
  //   }

    // float temperature = -46.85 + (175.72 / 65536.0) * float(raw_data[0]);
    // float humidity = -6.0 + (125.0 / 65536.0) * float(raw_humidity[1]);

  //   ESP_LOGD(TAG, "Got temperature=%.2f°C humidity=%.2f%%", temperature, humidity);
  //   if (this->temperature_sensor_ != nullptr)
  //     this->temperature_sensor_->publish_state(temperature);
  //   if (this->humidity_sensor_ != nullptr)
  //     this->humidity_sensor_->publish_state(humidity);
  //   this->status_clear_warning();
  // });
}

}  // namespace sht2x
}  // namespace esphome
