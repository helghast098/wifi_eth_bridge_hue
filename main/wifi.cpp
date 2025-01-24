#include <iostream>
#include <format>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_netif.h"
#include "lwip/sys.h"
#include "lwip/err.h"
#include "esp_eth_driver.h"
#include "esp_private/wifi.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT BIT1
#define DHCPS_OFFER_DNS 0x02

#define SSID_NAME "Franklin T10 9300"
#define STA_PASSWORD "20e86e12"

#define MAC_2_STR(a) std::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5])

static EventGroupHandle_t wifi_eventgroup;
static const char *TAG = "wifi AP Ex";
static const char *STA_TAG = "Station Info";


void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{

    switch ( event_id )
    {
        case WIFI_EVENT_AP_START: // AP IF started
        {
            std::cout << "AP STARTED";
            break;
        }

        case WIFI_EVENT_AP_STACONNECTED: // Station connected to AP
        {
            auto event_d = static_cast< wifi_event_ap_staconnected_t* >( event_data );

            std::cout << "AP:  Device Connected - "<< "MAC ADDRESS: " << MAC_2_STR( event_d->mac ) << "  " << "AID: " << event_d->aid << std::endl;
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: // Station disconnected from AP
        {
            auto event_d = static_cast< wifi_event_ap_stadisconnected_t* >( event_data );
            std::cout << "AP:  Device Disconnected - "<< "MAC ADDRESS: " << MAC_2_STR( event_d->mac ) << "  " << "AID: " <<\
                 event_d->aid << "  " << "Reason: " << event_d->reason << std::endl;
            break;
        }

        case WIFI_EVENT_STA_START: // Starting STA
        {
            std::cout << "Starting Station" << std::endl;
            esp_wifi_connect();
            break;
        }

        default:
            std::cout << "ESP BASE: " << event_base << "  " << "ID: " << event_id << std::endl;
    }

}

void ip_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch( event_id )
    {
        case IP_EVENT_STA_GOT_IP: // Esp connected to wifi
        {
            auto ip_addr = static_cast<ip_event_got_ip_t* >(event_data);
            ip_addr->ip_info. 
            std::cout << "STA: Got IP: " IPSTR << std::endl;
        }
    }
}


esp_netif_t* softap_init()
{
    // Creating default netif for ap
    esp_netif_t* ap_netif_handle = esp_netif_create_default_wifi_ap();

    // Setting SoftAp: wifi

    wifi_config_t ap_wifi_cfg = {
        .ap = {
            .ssid = "My Wifi",
            .password = "Corona78",
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 2,
            .beacon_interval = 100, // Default Value 0.1024s
            .pmf_cfg = {
                .required = false,
            },
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH
        }
    };

    ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_AP, &ap_wifi_cfg ) );

    return ap_netif_handle;
}

esp_err_t my_wifi_rxcb(void *buffer, uint16_t len, void *eb)
{

    // Transmit info to ethernet
    esp_err_t  err = ESP_OK;
    esp_wifi_internal_free_rx_buffer(eb);
    return err;
}


esp_netif_t* sta_init()
{
    // Creating Netif for STA
    esp_netif_t* sta_netif_handle = esp_netif_create_default_wifi_sta();

    // Setting Station Settings
    wifi_config_t sta_wifi_cfg = {
        .sta = {
            .ssid = SSID_NAME,
            .password = STA_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .failure_retry_cnt = 3
        }
    };

    ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &sta_wifi_cfg ) );

    esp_wifi_internal_reg_rxcb(WIFI_IF_STA, nullptr);

    return sta_netif_handle;
}

void  ap_set_dns_addr(esp_netif_t *sta_netif_handle, esp_netif_t *ap_netif_handle)
{

    // Get the dns info from sta
    esp_netif_dns_info_t sta_dns_info;

    ESP_ERROR_CHECK( esp_netif_get_dns_info( sta_netif_handle,  ESP_NETIF_DNS_MAIN, &sta_dns_info ) );

    // Setup softap dhcps
    ESP_ERROR_CHECK( esp_netif_dhcps_stop( ap_netif_handle ) );

    uint8_t set_custom_dns = DHCPS_OFFER_DNS;
    ESP_ERROR_CHECK( esp_netif_dhcps_option( ap_netif_handle, ESP_NETIF_OP_SET,  ESP_NETIF_DOMAIN_NAME_SERVER, &set_custom_dns, sizeof( set_custom_dns ) ) );

    // write the new dns info
    ESP_ERROR_CHECK( esp_netif_set_dns_info( ap_netif_handle, ESP_NETIF_DNS_MAIN, &sta_dns_info ) );

    // start the dhcp server
   ESP_ERROR_CHECK_WITHOUT_ABORT( esp_netif_dhcps_start( ap_netif_handle ) );
}

extern "C" void app_main(void)
{

    ESP_ERROR_CHECK(nvs_flash_init());

    // Setting up event loop
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    esp_event_handler_register( // Registering wifi event handler
        WIFI_EVENT, 
        ESP_EVENT_ANY_ID, 
        wifi_event_handler, 
        nullptr
    );

    esp_event_handler_register( // Registering IP event handler
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        ip_event_handler,
        nullptr
    );

    // Initializing TCP Stack using NETIF
    ESP_ERROR_CHECK( esp_netif_init() );


    // Config the Wifi
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK( esp_wifi_init( &wifi_init_cfg ) );

    esp_wifi_set_mode( WIFI_MODE_STA ); // Setting wifi as softap/sta mode

    // setting up network interfaces for softap and sta
    esp_netif_t *sta_netif_handle = sta_init();

    // Start the wifi
    ESP_ERROR_CHECK( esp_wifi_start() );
    printf("asdsadasd");

}
