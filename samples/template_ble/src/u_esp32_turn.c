#include <zephyr/zephyr.h>
#include <u_i2c_turn.h>
#include <u_esp32_turn.h>

#define ESP_32         0xC

volatile uint8_t msg_length;
bool esp_present;
typedef uint16_t ret_code_t;

uint8_t init_esp32(void)
{
  //ret_code_t err_code=0;
  //err_code = nrf_drv_twi_rx(&m_twi, ESP_32, &msg_length, sizeof(msg_length));
  //APP_ERROR_CHECK(err_code);

  if(!check_i2c_address(ESP_32))
  {
    esp_present = 0;
    return 1;
  }
  else
  {
    esp_present = 1;
    return 0;
  }
}

uint8_t time_request_esp32(uint8_t *aws_time)
{
    ret_code_t err_code=0;
    uint8_t esp_data[3] = {0x0F, 0x71,0x0E};
    uint8_t data_tx_len = sizeof(esp_data);
    uint8_t data_rx_len = 8;

    if(esp_present){
        
        err_code = u_i2c_write_read(ESP_32, esp_data, data_tx_len, aws_time, data_rx_len);
        //err_code = u_i2c_write_read(ESP_32, esp_data, data_tx_len, aws_time, data_rx_len);        

        return 0;
    }
    else
    {
        return -1;
    }
}

void get_len_esp32()
{
    ret_code_t err_code=0;
    uint8_t esp_data[3] = {0x0F, 0x72,0x0E};
    uint8_t data_tx_len = sizeof(esp_data);
    uint8_t data_rx_len = sizeof(msg_length);
    if(esp_present){

        err_code = u_i2c_write_read(ESP_32, esp_data, data_tx_len, msg_length, data_rx_len);
        //err_code = u_i2c_write_read(ESP_32, esp_data, data_tx_len, msg_length, data_rx_len);

    }
    return err_code;
}

// void get_msg(uint8_t *buf, uint8_t len)
// {
//   ret_code_t err_code=0;
//   uint8_t esp_data[3] = {0x0F, 0x70,0x0E};
//   if(esp_present){
//  err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, buf, len);
//   APP_ERROR_CHECK(err_code);

//   err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, buf, len);
//   APP_ERROR_CHECK(err_code);
//   }
// }


// void connect_status(uint8_t *connection)
// {
//   ret_code_t err_code=0;
//   uint8_t esp_data[3] = {0x0F, 0x73,0x0E};
//   if(esp_present){
//  err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, connection, 1);
//   APP_ERROR_CHECK(err_code);

//   err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, connection, 1);
//   APP_ERROR_CHECK(err_code);
// }
// }

// void get_wifi(uint8_t *wifi_rssi)
// {
//   ret_code_t err_code=0;
//   uint8_t esp_data[3] = {0x0F, 0x74,0x0E};
//   if(esp_present){
//  err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, wifi_rssi, 23);
//   APP_ERROR_CHECK(err_code);
//   nrf_delay_ms(40);
//   err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, wifi_rssi, 23);
//   APP_ERROR_CHECK(err_code);
// }
// }

// void get_mac()
// {
//   ret_code_t err_code=0;
//   uint8_t esp_data[3] = {0x0F, 0x75,0x0E};
//   if(esp_present){
//  err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, &this.bin_id, 6);
//   APP_ERROR_CHECK(err_code);

//   err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);


//   err_code = nrf_drv_twi_rx(&m_twi, ESP_32, &this.bin_id, 6);
//   APP_ERROR_CHECK(err_code);
// }
// }

// void esp_reset()
// {
//     ret_code_t err_code=0;
//   uint8_t esp_data[3] = {0x0F, 0x76,0x0E};
//   if(esp_present){
//  err_code = nrf_drv_twi_tx(&m_twi, ESP_32,&esp_data,sizeof(esp_data),false);
// }
// }