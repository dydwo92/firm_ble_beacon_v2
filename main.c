#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nordic_common.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"


/*********************************************************************************************/
//
// BLE FUNCTIONS
//
/*********************************************************************************************/
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier. */
#define APP_KITECH_IDENTIFIER           0x00, 0x00
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    0x02, 0x15, 0xD0, 0x6B, 0xDA, // Dummy
    0xD2, 0x7F, 0xE5, 0x4C, 0xDF, // Dummy
    0x89, 0x03, 0x06, 0x7A, 0x3F, // Dummy
    0x75, 0x67, 0xE4, // Dummy
    APP_KITECH_IDENTIFIER, // Product Identifier
    0x00, 0x00, // Encoder value
    0xFF, // Dummy
};

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

/*********************************************************************************************/
//
// SLEEP TIMER FUNCTIONS
//
/*********************************************************************************************/
static volatile uint8_t  m_sleep_phase;

APP_TIMER_DEF(m_detection_sleep_init_id);
APP_TIMER_DEF(m_detection_sleep_start_id);

static void sleep_init_handler(void* p_context){
  if(m_sleep_phase == 1 && bsp_board_button_state_get(0)){
    bsp_board_led_off(0);
    bsp_board_led_on(1);
    m_sleep_phase = 2;
    app_timer_start(m_detection_sleep_start_id, APP_TIMER_TICKS(3000), NULL);
  }else{
    m_sleep_phase = 0;
  }
}

static void sleep_start_handler(void* p_context){
  if(m_sleep_phase == 2){
    sd_power_system_off();
  }
}

static void sleep_button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
      bsp_board_led_on(0);
      bsp_board_led_off(1);
      m_sleep_phase = 1;
      app_timer_start(m_detection_sleep_init_id, APP_TIMER_TICKS(3000), NULL);
}

/*********************************************************************************************/
//
// SPI FUNCTIONS
//
/*********************************************************************************************/
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

uint8_t m_tx_buf[2];  /**< TX buffer. */
uint8_t m_rx_buf[2];  /**< RX buffer. */
uint8_t m_length = 2; /**< Transfer length. */

APP_TIMER_DEF(m_spi_loop_id);

static void spi_send_handler(void* p_context){
      memset(m_rx_buf, 0, m_length);
      spi_xfer_done = false;
      // Parity
      uint8_t v;
      uint16_t data = 0x8000 | (0x3FFF << 1);
      v = (uint8_t)(data ^ (data >> 8));
      v ^= v >> 4;
      v ^= v >> 2;
      v ^= v >> 1;
      if(v & 0x01) data |= 0x0001;
      m_tx_buf[0] = (uint8_t)(data >> 8);
      m_tx_buf[1] = (uint8_t)data;

      nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length);  
}

static void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void* p_context){
    uint16_t tmp;
    spi_xfer_done = true;
    if (m_rx_buf[0] != 0){
      tmp = (uint16_t)m_rx_buf[0] << 8 | (uint16_t)m_rx_buf[1];
      tmp = tmp >> 2;
      tmp = tmp & 0x03FF;

      // Write to BLE AdvData
      m_adv_data.adv_data.p_data[27] = (uint8_t)(tmp >> 8);
      m_adv_data.adv_data.p_data[28] = (uint8_t)tmp;
    }
}

/*********************************************************************************************/
// BSP INIT
/*********************************************************************************************/
static void bsp_configuration(){

    // [[ Enable sleep/wakeup button interrupt ]]
    if(!nrf_drv_gpiote_is_init()) nrf_drv_gpiote_init();

    nrf_drv_gpiote_in_config_t config = {
      .sense = GPIOTE_CONFIG_POLARITY_LoToHi,
      .pull = NRF_GPIO_PIN_NOPULL,
      .is_watcher = false,
      .hi_accuracy = false,
      .skip_gpio_setup = false,
    };
    nrf_drv_gpiote_in_init(bsp_board_button_idx_to_pin(0), &config, sleep_button_handler);

    app_timer_create(&m_detection_sleep_init_id,
                     APP_TIMER_MODE_SINGLE_SHOT,
                     sleep_init_handler);

    app_timer_create(&m_detection_sleep_start_id,
                     APP_TIMER_MODE_SINGLE_SHOT,
                     sleep_start_handler);

    nrf_drv_gpiote_in_event_enable(bsp_board_button_idx_to_pin(0), true);

    // [[ Enable wakeup indicate led & sensor power gpio ]]
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_on(0); // Indicator on
    bsp_board_led_off(1); // Sensor power on
                                  
    // [[ Init SPI ]]
    nrf_drv_spi_config_t spi_config = {                                                            
      .sck_pin      = SPI_SCK_PIN,                
      .mosi_pin     = SPI_MOSI_PIN,                
      .miso_pin     = SPI_MISO_PIN,                
      .ss_pin       = SPI_SS_PIN,                
      .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         
      .orc          = 0xFF,                                    
      .frequency    = NRF_DRV_SPI_FREQ_1M,                     
      .mode         = NRF_DRV_SPI_MODE_1,                      
      .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         
    };
    nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);

    app_timer_create(&m_spi_loop_id,
                     APP_TIMER_MODE_REPEATED,
                     spi_send_handler);

    app_timer_start(m_spi_loop_id, APP_TIMER_TICKS(50), NULL);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    APP_ERROR_CHECK(app_timer_init());
    bsp_configuration();
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
    ble_stack_init();
    advertising_init();

    // Start execution.
    advertising_start();

    // Enter main loop.
    while(1)
    {    
        nrf_pwr_mgmt_run();
    }
}


/**
 * @}
 */
