/*Freertos lib*/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>

/*std lib*/
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <atomic>

/*esp lib*/
#include "esp_types.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* DEFINES */
#define SSID_NAME CONFIG_WIFI_NAME
#define STA_PASSWORD CONFIG_WIFI_PASSWORD
#define WIFI_IP_BIT BIT0

#define ETH_QUEUE_LEN 100
#define ETH_QUEUE_TIMEOUT 100
#define WIFI_IP_TIMEOUT 10000

/* const vars */
const char* TAG = "MY ETHERNET";
const char* TAG2 = "MY WIFI";

/* file vars */
static EventGroupHandle_t wifi_eventgroup;

static QueueHandle_t eth_packet_queue;

static esp_eth_handle_t eth_driver_handle;

static uint8_t connected_device_mac[6] = {0};

static std::atomic_bool wifi_connected = false;
static std::atomic_bool ethernet_connected = false;
static std::atomic_bool wifi_ip_addr_got = false;

struct PacketData
{
    // Constructor
    PacketData(): packet(nullptr), len(0) {}
    PacketData(uint8_t *_packet, size_t _len): packet(_packet), len(_len){}

    // Data
    uint8_t *packet;
    size_t len;
};

/*---------- fn: Task to transmit ETH to wifi ----------*/
void eth_transmit_task( void *argv )
{
    PacketData packet;

    while( 1 ) 
    {
        if ( xQueueReceive( eth_packet_queue, &packet, portMAX_DELAY ) )
        {
        printf( "|------\n" );
            if ( !wifi_connected )
            {
                // need to set wifi mac address similar to device mac
                memcpy( connected_device_mac, ( uint8_t* )packet.packet + 6, sizeof( connected_device_mac ) );
                esp_wifi_set_mac( WIFI_IF_STA, connected_device_mac );

                // start and connect wifi
                esp_wifi_start();
                esp_wifi_connect();

                xEventGroupWaitBits(
                    wifi_eventgroup,
                    WIFI_IP_BIT,
                    pdTRUE,
                    pdFALSE,
                    pdMS_TO_TICKS( WIFI_IP_TIMEOUT )
                );

            }

            if ( wifi_ip_addr_got )
            {
                // send to wifi
                esp_err_t res = esp_wifi_internal_tx( WIFI_IF_STA, packet.packet, packet.len );

                if (res != ESP_OK)
                {
                    ESP_LOGW(TAG2, "Packet not sent to wifi\n");
                }
            }

            // free the packet
            free( packet.packet );
        }

    }

}

/*---------- fn: rxcb and ETH Stack functions ----------*/
esp_err_t my_wifi_rxcb(void *buffer, uint16_t len, void *eb)
{

    // Transmit info to ethernet
    esp_err_t  err = ESP_OK;
    if ( ethernet_connected )
    {
        err = esp_eth_transmit( eth_driver_handle, buffer, len );
        if ( err != ESP_OK)
        {
            ESP_LOGE( TAG2, "Failed to transmit to ethernet\n" );
        }
    }

    esp_wifi_internal_free_rx_buffer( eb );
    return err;
}

esp_err_t my_eth_stack_input(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv)
{
    PacketData packet(buffer, length);

    esp_err_t res = xQueueSend( eth_packet_queue, &packet, pdMS_TO_TICKS( ETH_QUEUE_TIMEOUT ) );

    if (res == errQUEUE_FULL)
    {
        ESP_LOGW(TAG, "Queue is Full");
        free(buffer);
    }

    return res != pdTRUE ? ESP_FAIL : ESP_OK;
}

/*---------- fn: Wifi and ETH event handlers ----------*/
// ETH
void eth_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch( event_id )
    {
     /* Ethernet start */
        case ETHERNET_EVENT_START:
        {
            ESP_LOGI(TAG, "Ethernt Started\n");
            break;
        }

        /* Ethernet Stopped */
        case ETHERNET_EVENT_STOP:
        {
            ESP_LOGI(TAG, "Ethernet Stopped\n");
            break;
        }

        /* Ethernet Connected */
        case ETHERNET_EVENT_CONNECTED:
        {
            
            ESP_LOGI(TAG, "Ethernet: A Device Connected");
            ethernet_connected = true;

            break;
        }

        case ETHERNET_EVENT_DISCONNECTED:
        {
            ESP_LOGI(TAG, "Ethernet: A Device Disconnected\n");

            ethernet_connected = false;

            esp_wifi_stop();

            wifi_connected = false;
        }
        default:
        ;
    }
}
// WIFI
void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{

    switch ( event_id )
    {

        case WIFI_EVENT_STA_START: // Starting STA
        {
            ESP_LOGI(TAG2, "Starting Station\n");
            //ESP_ERROR_CHECK(esp_wifi_connect());

            break;
        }

        case WIFI_EVENT_STA_STOP:
        {
            ESP_LOGI(TAG2, "Wifi Stopped\n");
            break;
        }

        case WIFI_EVENT_STA_CONNECTED:
        {
            ESP_LOGI(TAG2, "Wifi Connected to AP\n");
            esp_wifi_internal_reg_rxcb(WIFI_IF_STA, my_wifi_rxcb);
            wifi_connected = true;
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            ESP_LOGI(TAG2, "Wifi Disconnected from AP\n");
            wifi_connected = false;
            esp_wifi_internal_reg_rxcb(WIFI_IF_STA, nullptr);
            wifi_ip_addr_got = false;
            break;
        }
        default:
            ;
    }

}
//WIFI_IP
void wifi_ip_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch( event_id )
    {
        case IP_EVENT_STA_GOT_IP: // Esp connected to wifi
        {
            auto ip = static_cast<ip_event_got_ip_t* >(event_data);
            ESP_LOGI(TAG2, "STA: GOT IP: " IPSTR "\n", IP2STR(&ip->ip_info.ip));
            //xEventGroupSetBits(wifi_eventgroup, WIFI_IP_BIT);
            wifi_ip_addr_got = true;
        }
        default:
        ;
    }
}

/*---------- fn: Ethernet Init ----------*/
esp_eth_handle_t eth_init()
{
/*Setting Up EMAC */
ESP_LOGI( TAG, "========== MAC and EMAC Setup ==========\n" );

    ESP_LOGI( TAG, "Setting up EMAC" );
    // Setting up emac
    eth_esp32_emac_config_t esp_emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;

    ESP_LOGI( TAG, "Setting up MAC" );
    // Setting up mac
    eth_mac_config_t eth_mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_mac_cfg.sw_reset_timeout_ms = 100;

    esp_eth_mac_t *eth_mac_instance = esp_eth_mac_new_esp32( &esp_emac_cfg, &eth_mac_cfg );

    if ( eth_mac_instance == nullptr )
    {
        ESP_LOGE( TAG, "Issue creating ESP32 MAC Instance" );
        vTaskDelay( portMAX_DELAY );
    }

ESP_LOGI( TAG, "********** END MAX and EMAC Setup **********\n" );

/*Setting up PHY*/
ESP_LOGI( TAG, "========== PHY Setup ==========" );

    ESP_LOGI(TAG, "Setting up PHY");
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = 1;
    phy_cfg.reset_gpio_num = -1;
    
    esp_eth_phy_t* phy_instance = esp_eth_phy_new_lan87xx( &phy_cfg );

    if ( phy_instance == nullptr )
    {
        ESP_LOGE( "ERROR", "Issue creating ESP32 PHY Instance" );
        vTaskDelay(portMAX_DELAY);
    }

ESP_LOGI( TAG, "********** END PHY Setup **********\n" );

/*Setting up external clock */
ESP_LOGI( TAG, "========== External Clock Setup ==========" );

    ESP_LOGI( TAG, "Turning on External Clock" );
    ESP_ERROR_CHECK( gpio_set_direction( GPIO_NUM_16, GPIO_MODE_OUTPUT ) );
    ESP_ERROR_CHECK( gpio_set_level(  GPIO_NUM_16, 1 ) );

ESP_LOGI( TAG, "********** End External Clock Setup **********\n" );


/*Installing Ethernet Driver*/
ESP_LOGI( TAG, "========== Installing Ethernet Driver ==========" );
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG( eth_mac_instance, phy_instance ); // default driver configs
    esp_eth_handle_t eth_driver_handle = nullptr; // driver handle

    ESP_ERROR_CHECK( esp_eth_driver_install( &eth_cfg, &eth_driver_handle ) ); // install driver

    esp_eth_update_input_path( eth_driver_handle, my_eth_stack_input, nullptr );  // set own input stack func.

    bool promiscuous_true = true;
   esp_eth_ioctl( eth_driver_handle, ETH_CMD_S_PROMISCUOUS, &promiscuous_true ); // Make Ethernet Listen to all traffic
ESP_LOGI( TAG, "********** End Installing Ethernet Driver **********\n" );


    ESP_ERROR_CHECK( esp_eth_start( eth_driver_handle ) );


/*Setting up event group */
    esp_event_handler_instance_register(
        ETH_EVENT,
        ESP_EVENT_ANY_ID,
        eth_event_handler,
        nullptr,
        nullptr
        );

    //return ethernet driver handle;
    return eth_driver_handle;
}

esp_netif_t* sta_init()
{
    // Creating Netif for STA

    // Initializing wifi for STA Mode
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &wifi_init_cfg ) );
    esp_wifi_set_mode( WIFI_MODE_STA );

    esp_netif_t* sta_netif_handle = esp_netif_create_default_wifi_sta();
    // Setting Station Settings
    wifi_config_t sta_wifi_cfg = {
        .sta = {
            .ssid = SSID_NAME,
            .password = STA_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = 3
        }
    };

    ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &sta_wifi_cfg ) );


    // Wifi event handlers
    esp_event_handler_register( // Registering wifi event handler
        WIFI_EVENT, 
        ESP_EVENT_ANY_ID, 
        wifi_event_handler, 
        nullptr
    );

    esp_event_handler_register( // Registering IP event handler
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_ip_event_handler,
        nullptr
    );

    // Wifi event group
    wifi_eventgroup = xEventGroupCreate();
    
    return sta_netif_handle;
}

/*---------- fn: main ----------*/
extern "C" void app_main()
{
    if ( nvs_flash_init() != ESP_OK )
    {
        // clear nvs_flash
        nvs_flash_erase();

        // re initialize
        nvs_flash_init();
    }

/*Creating queue */
    eth_packet_queue = xQueueCreate( ETH_QUEUE_LEN, sizeof( PacketData ) );

    esp_netif_init();

/*Starting Default Event Loop */
    esp_event_loop_create_default();

/*Initializing Wifi */
    sta_init();

/*Initializing Ethernet*/
    eth_driver_handle = eth_init();

    xTaskCreate(
        eth_transmit_task,
        "Eth Transmit Task",
        3000,
        nullptr,
        2,
        nullptr
    );
}