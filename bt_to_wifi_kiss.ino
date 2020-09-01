
#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
//#include <WebServer.h>

#include <stdbool.h>
#include <stdint.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "wifi_password.h"


static const char*    ssid     = STASSID;
static const char*    password = STAPSK;

#define KISS_PORT   8001
#define DEVICE_NAME "Far-Out-Labs"
 
static const  esp_spp_mode_t Esp_spp_mode  =  ESP_SPP_MODE_CB ; 
static const  esp_spp_sec_t  Sec_mask      =  ESP_SPP_SEC_AUTHENTICATE ; 
static const  esp_spp_role_t Role_master   =  ESP_SPP_ROLE_MASTER ;

static  uint8_t        Peer_bdname_len ; 
static  char           Peer_bdname [ ESP_BT_GAP_MAX_BDNAME_LEN  +  1 ]; 
static  esp_bd_addr_t  Peer_bd_addr  =  { 0x04 , 0xEE , 0x03 , 0x9F , 0xB0 , 0x8C };

static const  esp_bt_inq_mode_t  Inq_mode     = ESP_BT_INQ_MODE_GENERAL_INQUIRY; 
static const  uint8_t            Inq_len      = 30; 
static const  uint8_t            inq_num_rsps = 0;


static WiFiClient* s_currentClient = NULL; // we only support one client at the moment (which is fine for this project)

WiFiServer wifiServer( KISS_PORT );


bool  get_name_from_eir ( uint8_t* eir, char* bdname, uint8_t* bdname_len ) 
{ 
    uint8_t* rmt_bdname =  NULL; 
    uint8_t  rmt_bdname_len  =  0 ;

    if( !eir )  
        return  false ; 
   
    rmt_bdname = esp_bt_gap_resolve_eir_data( eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,  &rmt_bdname_len ); 
    if( ! rmt_bdname )
        rmt_bdname = esp_bt_gap_resolve_eir_data( eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len ); 
    

    if( rmt_bdname )  
    { 
        if( rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN )  
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN; 

        if( bdname )  
        { 
            memcpy ( bdname,  rmt_bdname,  rmt_bdname_len ); 
            bdname[rmt_bdname_len] = '\0'; 
        } 
        
        if( bdname_len ) 
            *bdname_len = rmt_bdname_len; 
        
        return true; 
    }

    return false; 
}


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char buf[1024];
    char spp_data[256];
    
    switch( event ) 
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name( DEVICE_NAME );
        esp_bt_gap_set_scan_mode( ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE );
        esp_bt_gap_start_discovery( Inq_mode , Inq_len, inq_num_rsps );
        break;
        
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        if( param->disc_comp.status == ESP_SPP_SUCCESS )   
            esp_spp_connect( Sec_mask,  Role_master, param->disc_comp.scn[0], Peer_bd_addr ); 
        break;
        
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
//        if (param->data_ind.len < 1023) {
//            snprintf(buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
//            printf("%s\n", buf);
//            sprintf(spp_data, "Receined characters: %d\n", param->data_ind.len);
//            esp_spp_write(param->write.handle, strlen(spp_data), (uint8_t *)spp_data);
//        }
//        else {
//            esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
//        }

          ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d", param->data_ind.len, param->data_ind.handle);
          
//          for( int i = 0; i < param->data_ind.len; i++ ) 
//          { 
//              if( param->data_ind.data[i] == '\r' || param->data_ind.data[i] == '\n' )  
//                Serial.print( "<n>" ); 
//              else 
//              {
//                if( param->data_ind.data[i] >= 32 && param->data_ind.data[i] <= 126 )
//                  Serial.print( (char)param->data_ind.data[i] ); 
//                else
//                  Serial.print( param->data_ind.data[i], HEX ); 
//              }
//          } 
//          Serial.println();


          if( s_currentClient )
          {
//            for( int i = 0; i < param->data_ind.len; i++ ) 
//            { 
//                if( param->data_ind.data[i] >= 32 && param->data_ind.data[i] <= 126 )
//                  s_currentClient->write( (char)param->data_ind.data[i] ); 
//                else
//                  s_currentClient->write( param->data_ind.data[i] ); 
//            } 
//            s_currentClient->write( '\n' );
              s_currentClient->write( param->data_ind.data, param->data_ind.len );
          }
          break;
        
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    default:
        break;
    }
}




void esp_bt_gap_cb( esp_bt_gap_cb_event_t event,  esp_bt_gap_cb_param_t* param ) 
{ 
    switch( event ) 
    { 
      case ESP_BT_GAP_DISC_RES_EVT: 
//        Serial.print( "ESP_BT_GAP_DISC_RES_EVT: num = " ); 
//        Serial.println( param->disc_res.num_prop ); 
        
        for ( int i = 0; i < param->disc_res.num_prop;  i++ ) 
        { 
            if( param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_COD && (memcmp( Peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN )  ==  0)  )  
            { 
              Serial.println( "Target device found..." );                 
              esp_spp_start_discovery( Peer_bd_addr ); 
              esp_bt_gap_cancel_discovery();
            } 
            
            if( param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_COD )  
            { 
              // Class Of Device 
              if( get_name_from_eir( (uint8_t*)(param->disc_res.prop[i].val), Peer_bdname,  &Peer_bdname_len )  )  
              { 
                Peer_bdname[Peer_bdname_len] = 0; 
                Serial.print( "PEER CLASS OF DEVICE: " );              
                Serial.println( Peer_bdname );              
              } 
            } 
            else if( param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_RSSI ) 
            { 
//              Serial.print( "ESP_BT_GAP_DEV_PROP_RSSI- param->disc_res.prop[i].type: " ); 
//              Serial.println( param->disc_res.prop[i].type ); 
            }
            else if( param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME )  
            { 
//              Serial.print( "ESP_BT_GAP_DEV_PROP_BDNAME- param->disc_res.prop[i].type: " ); 
//              Serial.println( param->disc_res.prop[i].type ); 
            } 
            else 
            { 
//              Serial.print( "param->disc_res.prop[i].type: " ); 
//              Serial.println( param->disc_res.prop[i].type ); 
                Serial.println( "." );
            } 
        } 
        break; 
        
    case  ESP_BT_GAP_DISC_STATE_CHANGED_EVT : 
        break ; 
    case  ESP_BT_GAP_RMT_SRVCS_EVT : 
        break ; 
    case  ESP_BT_GAP_RMT_SRVC_REC_EVT : 
        break ; 
    case  ESP_BT_GAP_AUTH_CMPL_EVT :
        break ; 
   
    case  ESP_BT_GAP_PIN_REQ_EVT : 
    { 
        if( param->pin_req.min_16_digit )  
        { 
            esp_bt_pin_code_t  Pin_code  =  { 0 }; 
            esp_bt_gap_pin_reply( param->pin_req.bda, true, 16, Pin_code ); 
        }  
        else  
        { 
            esp_bt_pin_code_t  Pin_code ; 
            Pin_code[0] = '1'; 
            Pin_code[1] = '2'; 
            Pin_code[2] = '3'; 
            Pin_code[3] = '4'; 
            esp_bt_gap_pin_reply( param->pin_req.bda, true, 4, Pin_code ); 
        } 
        break ; 
    } 
    case  ESP_BT_GAP_CFM_REQ_EVT : 
        esp_bt_gap_ssp_confirm_reply( param->cfm_req.bda, true ); 
        break ; 
    
    case  ESP_BT_GAP_KEY_NOTIF_EVT : 
        break ; 
    case  ESP_BT_GAP_KEY_REQ_EVT : 
        break ; 
    default:
        break ; 
    } 
}


void setup() 
{
    Serial.begin( 115200 );  
    Serial.println( "Connecting to WiFi" );

    WiFi.mode( WIFI_STA );
    WiFi.begin( ssid, password );

    while( WiFi.status() != WL_CONNECTED ) 
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println( "WiFi connected: " );
    Serial.println( WiFi.localIP() );

    if( MDNS.begin( DEVICE_NAME ) ) 
    {
      Serial.println( "mDNS responder started..." );
      MDNS.addService( "kiss", "_tcp", KISS_PORT );
    }

    wifiServer.begin();
    
    esp_err_t ret = 0;
//    esp_err_t ret = nvs_flash_init();
//    if( ret == ESP_ERR_NVS_NO_FREE_PAGES ) 
//    {
//        ESP_ERROR_CHECK( nvs_flash_erase() );
//        ret = nvs_flash_init();
//    }
//    ESP_ERROR_CHECK( ret );


    if( !btStart() )
      ret = ESP_FAIL;
      
    if( ret != ESP_OK ) 
    {
        ESP_LOGE(SPP_TAG, "%s initialize bluetooth failed: %d\n", __func__, ret );
        return;
    }

    esp_bluedroid_status_t Bt_state = esp_bluedroid_get_status(); 
    if( Bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED ) 
    { 
        if ( esp_bluedroid_init() )  
        { 
            Serial.println( "the initialize Bluedroid failed" ); 
            return; 
        } 
    }

    if( Bt_state != ESP_BLUEDROID_STATUS_ENABLED ) 
    { 
        if( esp_bluedroid_enable() )  
        { 
            Serial.println( "Enable Bluedroid Failed" ); 
            return; 
        } 
    }

    if( (ret = esp_bt_gap_register_callback( esp_bt_gap_cb ))  !=  ESP_OK ) 
        return; 

    if( (ret = esp_spp_register_callback( esp_spp_cb )) != ESP_OK )
        return; 

    if( (ret = esp_spp_init( Esp_spp_mode )) != ESP_OK ) 
        return; 
 
    /* Set Default Parameters For Secure Simple Pairing */ 
    esp_bt_sp_param_t  Param_type = ESP_BT_SP_IOCAP_MODE; 
    esp_bt_io_cap_t    Iocap      = ESP_BT_IO_CAP_IO; 
    esp_bt_gap_set_security_param( Param_type, &Iocap, sizeof( uint8_t ) );

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */ 
    esp_bt_pin_type_t  pin_type  =  ESP_BT_PIN_TYPE_VARIABLE; 
    esp_bt_pin_code_t  pin_code; 
    esp_bt_gap_set_pin( pin_type,  0, pin_code );

    Serial.println( "Bluetooth init complete" );
}


void loop() 
{
  WiFiClient client = wifiServer.available();
 
  if( client )
  {
    Serial.println( "Client connected..." );
    s_currentClient = &client;
    while( client.connected() ) 
    {
      while( client.available() > 0 ) 
      {
        char c = client.read();
        client.write( c );
      }
 
      delay( 10 );
    }
    s_currentClient = NULL;
    client.stop();
    Serial.println( "Client disconnected..." );
  }
}
