/**
 * https://controllerstech.com/stm32-esp8266-wifi-ip/
 */

#include <esp32_at_stm32.h>

ESP8266_ConnectionState ESP_ConnState = ESP8266_DISCONNECTED;

// DMA buffers and variables
uint8_t esp_dma_rx_buffer[ESP_DMA_RX_BUFFER_SIZE];
uint8_t esp_dma_tx_buffer[ESP_DMA_TX_BUFFER_SIZE];
volatile uint16_t esp_dma_rx_head = 0;
volatile uint16_t esp_dma_rx_tail = 0;

// MQTT global variables
MQTT_ConnectionState mqtt_state = MQTT_DISCONNECTED;
MQTT_Config mqtt_config = {0};

// Legacy buffer
static char esp_rx_buffer[2048];

static ESP8266_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len);


void ESP_DMA_Init(void)
{
    // Initialize DMA buffers
    memset(esp_dma_rx_buffer, 0, ESP_DMA_RX_BUFFER_SIZE);
    memset(esp_dma_tx_buffer, 0, ESP_DMA_TX_BUFFER_SIZE);
    esp_dma_rx_head = 0;
    esp_dma_rx_tail = 0;

    DEBUG_LOG("DMA buffers initialized for USART1");
}

void ESP_DMA_StartReceive(void)
{
    // Start DMA reception in circular mode
    HAL_UART_Receive_DMA(&ESP_UART, esp_dma_rx_buffer, ESP_DMA_RX_BUFFER_SIZE);
    DEBUG_LOG("DMA reception started for USART1");
}

uint16_t ESP_DMA_GetReceivedData(uint8_t *buffer, uint16_t max_len)
{
    uint16_t dma_head = ESP_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    uint16_t data_len = 0;

    if (dma_head != esp_dma_rx_tail)
    {
        if (dma_head > esp_dma_rx_tail)
        {
            data_len = dma_head - esp_dma_rx_tail;
            if (data_len > max_len)
                data_len = max_len;
            memcpy(buffer, &esp_dma_rx_buffer[esp_dma_rx_tail], data_len);
            
            // Debug received data only when we actually get data
            if (data_len > 0)
            {
                buffer[data_len] = '\0';
                DEBUG_LOG("RX Data: %s (len=%d)", buffer, data_len);
            }
        }
        else
        {
            // Handle circular buffer wrap-around
            uint16_t first_part = ESP_DMA_RX_BUFFER_SIZE - esp_dma_rx_tail;
            uint16_t second_part = dma_head;

            if (first_part <= max_len)
            {
                memcpy(buffer, &esp_dma_rx_buffer[esp_dma_rx_tail], first_part);
                if (second_part <= (max_len - first_part))
                {
                    memcpy(&buffer[first_part], esp_dma_rx_buffer, second_part);
                    data_len = first_part + second_part;
                }
                else
                {
                    memcpy(&buffer[first_part], esp_dma_rx_buffer, max_len - first_part);
                    data_len = max_len;
                }
            }
            else
            {
                memcpy(buffer, &esp_dma_rx_buffer[esp_dma_rx_tail], max_len);
                data_len = max_len;
            }
            
            // Debug received data only when we actually get data
            if (data_len > 0)
            {
                buffer[data_len] = '\0';
                DEBUG_LOG("RX Data (wrapped): %s (len=%d)", buffer, data_len);
            }
        }

        esp_dma_rx_tail = (esp_dma_rx_tail + data_len) % ESP_DMA_RX_BUFFER_SIZE;
    }

    return data_len;
}

ESP8266_Status ESP_DMA_SendCommand(const char *cmd, const char *ack, uint32_t timeout)
{
    uint32_t tickstart;
    int found = 0;
    uint16_t cmd_len = strlen(cmd);
    uint8_t temp_buffer[1024];
    uint16_t total_received = 0;

    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    tickstart = HAL_GetTick();

    if (cmd_len > 0)
    {
//        DEBUG_LOG("DMA sending: %s", cmd);

        // Copy command to DMA TX buffer
        if (cmd_len > ESP_DMA_TX_BUFFER_SIZE)
            cmd_len = ESP_DMA_TX_BUFFER_SIZE;
        memcpy(esp_dma_tx_buffer, cmd, cmd_len);

        // Check UART state before transmission
        HAL_UART_StateTypeDef uart_state = HAL_UART_GetState(&ESP_UART);
        DEBUG_LOG("UART state before TX: %lu", uart_state);

        if (HAL_UART_Transmit_DMA(&ESP_UART, esp_dma_tx_buffer, cmd_len) != HAL_OK)
        {
            DEBUG_LOG("DMA transmission failed");
            return ESP8266_ERROR;
        }

        // Wait for transmission to complete
        while (HAL_UART_GetState(&ESP_UART) == HAL_UART_STATE_BUSY_TX)
        {
            if ((HAL_GetTick() - tickstart) > timeout)
            {
                DEBUG_LOG("TX timeout");
                return ESP8266_TIMEOUT;
            }
        }
        DEBUG_LOG("TX completed");
    }

    // Reset tick for reception timeout
    tickstart = HAL_GetTick();

    while ((HAL_GetTick() - tickstart) < timeout && total_received < sizeof(esp_rx_buffer) - 1)
    {
        uint16_t received = ESP_DMA_GetReceivedData(temp_buffer, sizeof(temp_buffer));

        if (received > 0)
        {
            // Copy to legacy buffer for compatibility
            if (total_received + received < sizeof(esp_rx_buffer) - 1)
            {
                memcpy(&esp_rx_buffer[total_received], temp_buffer, received);
                total_received += received;
                esp_rx_buffer[total_received] = '\0';

                // Check for ACK
                if (!found && strstr(esp_rx_buffer, ack))
                {
                    DEBUG_LOG("DMA Matched ACK: %s", ack);
                    found = 1;
                }

                // Handle busy response
                if (strstr(esp_rx_buffer, "busy"))
                {
                    DEBUG_LOG("ESP is busy... delaying before retry");
                    HAL_Delay(1500);
                    total_received = 0;
                    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
                    continue;
                }
            }
        }

        HAL_Delay(10); // Small delay to prevent busy waiting
    }

    if (found)
    {
        DEBUG_LOG("DMA Full buffer: %s", esp_rx_buffer);
        return ESP8266_OK;
    }

    if (total_received == 0)
        return ESP8266_NO_RESPONSE;

    DEBUG_LOG("DMA Timeout or no ACK. Buffer: %s", esp_rx_buffer);
    return ESP8266_TIMEOUT;
}

ESP8266_Status ESP_TestBasicUART(void)
{
    uint8_t rx_buffer[64];
    uint8_t tx_data[] = "AT\r\n";
    HAL_StatusTypeDef status;
    
    USER_LOG("Testing basic UART communication (non-DMA)...");
    
    // Clear RX buffer
    memset(rx_buffer, 0, sizeof(rx_buffer));
    
    // Send AT command using blocking transmission
    status = HAL_UART_Transmit(&ESP_UART, tx_data, strlen((char*)tx_data), 1000);
    if (status != HAL_OK)
    {
        DEBUG_LOG("UART TX failed: %d", status);
        return ESP8266_ERROR;
    }
    
    DEBUG_LOG("UART TX completed - sent: %s", tx_data);
    
    // Try to receive response using blocking reception
    status = HAL_UART_Receive(&ESP_UART, rx_buffer, sizeof(rx_buffer)-1, 3000);
    
    if (status == HAL_OK)
    {
        rx_buffer[sizeof(rx_buffer)-1] = '\0';
        DEBUG_LOG("UART RX success: %s", rx_buffer);
        
        if (strstr((char*)rx_buffer, "OK"))
        {
            USER_LOG("Basic UART communication working!");
            return ESP8266_OK;
        }
    }
    else if (status == HAL_TIMEOUT)
    {
        DEBUG_LOG("UART RX timeout - ESP32 not responding");
        DEBUG_LOG("Check: 1) Power (3.3V not 5V), 2) Wiring, 3) AT firmware");
    }
    else
    {
        DEBUG_LOG("UART RX error: %d", status);
    }
    
    return ESP8266_ERROR;
}

ESP8266_Status ESP_DetectBaudRate(void)
{
    uint32_t baud_rates[] = {115200, 9600, 38400, 57600, 460800, 921600};
    uint8_t num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
    ESP8266_Status res;
    
    USER_LOG("Detecting ESP32 baud rate...");
    
    for (uint8_t i = 0; i < num_rates; i++)
    {
        USER_LOG("Testing baud rate: %lu", baud_rates[i]);
        
        // Reconfigure UART with new baud rate
        HAL_UART_DeInit(&ESP_UART);
        ESP_UART.Init.BaudRate = baud_rates[i];
        if (HAL_UART_Init(&ESP_UART) != HAL_OK)
        {
            DEBUG_LOG("Failed to set baud rate %lu", baud_rates[i]);
            continue;
        }
        
        // Test with basic UART first (more reliable)
        res = ESP_TestBasicUART();
        if (res == ESP8266_OK)
        {
            USER_LOG("Found working baud rate: %lu", baud_rates[i]);
            return ESP8266_OK;
        }
        
        // If basic UART fails, try DMA
        ESP_DMA_StartReceive();
        HAL_Delay(500);
        
        // Test AT command with DMA
        res = ESP_DMA_SendCommand("AT\r\n", "OK", 2000);
        if (res == ESP8266_OK)
        {
            USER_LOG("Found working baud rate: %lu (DMA)", baud_rates[i]);
            return ESP8266_OK;
        }
        
        // Try alternative line endings
        res = ESP_DMA_SendCommand("AT\n", "OK", 2000);
        if (res == ESP8266_OK)
        {
            USER_LOG("Found working baud rate: %lu (with \\n)", baud_rates[i]);
            return ESP8266_OK;
        }
    }
    
    DEBUG_LOG("No working baud rate found");
    return ESP8266_ERROR;
}

ESP8266_Status ESP_Init(void)
{
    ESP8266_Status res;
    USER_LOG("Initializing ESP32 with DMA...");

    // Initialize DMA for USART1
    ESP_DMA_Init();
    ESP_DMA_StartReceive();

    HAL_Delay(2000); // Longer delay for ESP32 boot

    // First try to detect the correct baud rate
    res = ESP_DetectBaudRate();
    if (res != ESP8266_OK)
    {
        DEBUG_LOG("Baud rate detection failed, trying default settings...");
        
        // Try different AT command formats for ESP32 compatibility
        USER_LOG("Testing ESP32 AT communication with default baud...");
        
        // Test 1: Basic AT command
        res = ESP_DMA_SendCommand("AT\r\n", "OK", 3000);
        if (res != ESP8266_OK)
        {
            DEBUG_LOG("AT command failed, trying alternative formats...");
            
            // Test 2: Try without \r\n
            res = ESP_DMA_SendCommand("AT", "OK", 3000);
            if (res != ESP8266_OK)
            {
                // Test 3: Try with just \n
                res = ESP_DMA_SendCommand("AT\n", "OK", 3000);
                if (res != ESP8266_OK)
                {
                    // Test 4: Try with just \r
                    res = ESP_DMA_SendCommand("AT\r", "OK", 3000);
                    if (res != ESP8266_OK)
                    {
                        DEBUG_LOG("ESP32 Not Responding to any AT format...");
                        return res;
                    }
                }
            }
        }
    }

    USER_LOG("ESP32 AT communication established!");

    // Get ESP32 version info for debugging
    res = ESP_DMA_SendCommand("AT+GMR\r\n", "OK", 3000);
    if (res == ESP8266_OK)
    {
        DEBUG_LOG("ESP32 Version Info: %s", esp_rx_buffer);
    }

    res = ESP_DMA_SendCommand("ATE0\r\n", "OK", 2000); // Disable echo
    if (res != ESP8266_OK)
    {
        DEBUG_LOG("Disable echo Command Failed...");
        return res;
    }
    
    USER_LOG("ESP32 Initialized Successfully with DMA...");
    return ESP8266_OK;
}

ESP8266_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len)
{
    USER_LOG("Setting in Station Mode");
    // Set in Station Mode
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWMODE=1\r\n");

    ESP8266_Status result = ESP_DMA_SendCommand(cmd, "OK", 2000); // wait up to 2s
    if (result != ESP8266_OK)
    {
        USER_LOG("Station Mode Failed.");
        return result;
    }

    USER_LOG("Connecting to WiFi SSID: %s", ssid);
    // Send join command
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

    result = ESP_DMA_SendCommand(cmd, "WIFI CONNECTED", 10000); // wait up to 10s
    if (result != ESP8266_OK)
    {
        USER_LOG("WiFi connection failed.");
        ESP_ConnState = ESP8266_NOT_CONNECTED;
        return result;
    }

    USER_LOG("WiFi Connected. Waiting for IP...");
    ESP_ConnState = ESP8266_CONNECTED_NO_IP;
    // Fetch IP with retries inside ESP_GetIP
    result = ESP_GetIP(ip_buffer, buffer_len);
    if (result != ESP8266_OK)
    {
        USER_LOG("Failed to fetch IP. Status=%d", result);
        return result;
    }

    USER_LOG("WiFi + IP ready: %s", ip_buffer);
    return ESP8266_OK;
}

ESP8266_ConnectionState ESP_GetConnectionState(void)
{
    return ESP_ConnState;
}

static ESP8266_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len)
{
    DEBUG_LOG("fetching IP address...");

    for (int attempt = 1; attempt <= 3; attempt++)
    {
        ESP8266_Status result = ESP_DMA_SendCommand("AT+CIFSR\r\n", "OK", 5000);
        if (result != ESP8266_OK)
        {
            DEBUG_LOG("CIFSR failed on attempt %d", attempt);
            continue;
        }

        char *search = esp_rx_buffer;
        char *last_ip = NULL;

        while ((search = strstr(search, "STAIP,")) != NULL)
        {
            char *ip_start = strstr(search, "STAIP,\"");

            if (ip_start)
            {
                ip_start += 7;
                // strchr -> locate the first occurrence of a specific character within a given string
                char *end = strchr(ip_start, '"');
                if (end && ((end - ip_start) < buffer_len))
                {
                    last_ip = ip_start;
                }
            }
            search += 6;
        }

        if (last_ip)
        {
            char *end = strchr(last_ip, '"');
            strncpy(ip_buffer, last_ip, end - last_ip);
            ip_buffer[end - last_ip] = '\0';

            if (strcmp(ip_buffer, "0.0.0.0") == 0)
            {
                DEBUG_LOG("Attempt %d: IP not ready yet (0.0.0.0). Retrying...", attempt);
                ESP_ConnState = ESP8266_CONNECTED_NO_IP;
                HAL_Delay(1000);
                continue;
            }

            DEBUG_LOG("Got IP: %s", ip_buffer);
            ESP_ConnState = ESP8266_CONNECTED_IP;
            return ESP8266_OK;
        }

        DEBUG_LOG("Attempt %d: Failed to parse STAIP.", attempt);
        HAL_Delay(500);
    }

    DEBUG_LOG("Failed to fetch IP after retries.");
    ESP_ConnState = ESP8266_CONNECTED_NO_IP; // still connected, but no IP
    return ESP8266_ERROR;
}

// ========== MQTT DIAGNOSTIC FUNCTIONS ==========

ESP8266_Status ESP_MQTT_TestBrokerConnectivity(const char *broker, uint16_t port)
{
    char cmd[128];
    ESP8266_Status result;
    
    USER_LOG("Testing TCP connectivity to MQTT broker %s:%d", broker, port);
    
    // Test basic TCP connection to broker
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", broker, port);
    result = ESP_DMA_SendCommand(cmd, "CONNECT", 10000);
    
    if (result == ESP8266_OK) {
        USER_LOG("✅ TCP connection to broker successful");
        // Close the connection
        ESP_DMA_SendCommand("AT+CIPCLOSE\r\n", "OK", 3000);
        HAL_Delay(500);
        return ESP8266_OK;
    } else {
        USER_LOG("❌ TCP connection to broker failed");
        USER_LOG("Response: %s", esp_rx_buffer);
        return ESP8266_ERROR;
    }
}

ESP8266_Status ESP_MQTT_GetFirmwareInfo(void)
{
    ESP8266_Status result;
    
    USER_LOG("Getting ESP32 firmware information...");
    
    // Get AT version
    result = ESP_DMA_SendCommand("AT+GMR\r\n", "OK", 3000);
    if (result == ESP8266_OK) {
        USER_LOG("Firmware info: %s", esp_rx_buffer);
    }
    
    // Check available MQTT commands
    USER_LOG("Checking MQTT command support...");
    
    const char* mqtt_commands[] = {
        "AT+MQTTUSERCFG=?",
        "AT+MQTTCONN=?", 
        "AT+MQTTCLIENTID=?",
        "AT+MQTTPUB=?",
        "AT+MQTTSUB=?",
        "AT+MQTTCLEAN=?"
    };
    
    for (int i = 0; i < 6; i++) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "%s\r\n", mqtt_commands[i]);
        result = ESP_DMA_SendCommand(cmd, "OK", 2000);
        if (result == ESP8266_OK) {
            USER_LOG("✅ %s supported", mqtt_commands[i]);
        } else {
            USER_LOG("❌ %s not supported", mqtt_commands[i]);
        }
    }
    
    return ESP8266_OK;
}

// ========== MQTT IMPLEMENTATION ==========

ESP8266_Status ESP_MQTT_Init(const char *broker, uint16_t port, const char *client_id)
{
    USER_LOG("Initializing MQTT configuration...");
    
    // Clear configuration
    memset(&mqtt_config, 0, sizeof(mqtt_config));
    
    // Set broker and port
    strncpy(mqtt_config.broker, broker, sizeof(mqtt_config.broker) - 1);
    mqtt_config.port = port;
    
    // Set client ID
    strncpy(mqtt_config.client_id, client_id, sizeof(mqtt_config.client_id) - 1);
    
    // Set defaults
    mqtt_config.keepalive = 60;
    mqtt_config.clean_session = 1;
    
    mqtt_state = MQTT_DISCONNECTED;
    
    USER_LOG("MQTT configured: Broker=%s:%d, ClientID=%s", broker, port, client_id);
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_SetAuth(const char *username, const char *password)
{
    if (username != NULL)
    {
        strncpy(mqtt_config.username, username, sizeof(mqtt_config.username) - 1);
    }
    
    if (password != NULL)
    {
        strncpy(mqtt_config.password, password, sizeof(mqtt_config.password) - 1);
    }
    
    DEBUG_LOG("MQTT authentication set");
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_Connect(void)
{
    char cmd[256];
    ESP8266_Status result;
    
    USER_LOG("Connecting to MQTT broker...");
    mqtt_state = MQTT_CONNECTING;
    
    // Validate client ID according to MQTT spec
    if (ESP_MQTT_ValidateClientId(mqtt_config.client_id) != ESP8266_OK)
    {
        USER_LOG("Invalid client ID format");
        mqtt_state = MQTT_CONNECTION_REFUSED_IDENTIFIER;
        return ESP8266_ERROR;
    }
    
    // Step 1: Test server connectivity first
    USER_LOG("Testing server connectivity to %s:%d...", mqtt_config.broker, mqtt_config.port);
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", mqtt_config.broker, mqtt_config.port);
    result = ESP_DMA_SendCommand(cmd, "CONNECT", 10000);
    if (result == ESP8266_OK) {
        USER_LOG("✅ Server is reachable");
        // Close the test connection
        ESP_DMA_SendCommand("AT+CIPCLOSE\r\n", "OK", 3000);
        HAL_Delay(1000);
    } else {
        USER_LOG("❌ Server NOT reachable - check network/firewall");
        mqtt_state = MQTT_ERROR;
        return ESP8266_ERROR;
    }
    
    // Step 2: Skip MQTT user config and try direct connection
    USER_LOG("Trying direct MQTT connection without user config...");
    
    // Clean any existing MQTT state first
    ESP_DMA_SendCommand("AT+MQTTCLEAN=0\r\n", "OK", 3000);
    HAL_Delay(500);
    
    // Try direct connection
    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,0\r\n",
             mqtt_config.broker, mqtt_config.port);
    
    result = ESP_DMA_SendCommand(cmd, "+MQTTCONNECTED", 15000);
    if (result != ESP8266_OK) {
        // try with minimal user config
        USER_LOG("Direct connection failed, trying minimal config...");
        snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"esp32\",\"\",\"\",0,0,\"\"\r\n");
        result = ESP_DMA_SendCommand(cmd, "OK", 5000);
        if (result == ESP8266_OK) {
            snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,1\r\n",
                     mqtt_config.broker, mqtt_config.port);
            result = ESP_DMA_SendCommand(cmd, "+MQTTCONNECTED", 15000);
        }
    }
    if (result == ESP8266_OK) {
        USER_LOG("✅ MQTT connected successfully!");
        mqtt_state = MQTT_CONNECTED;
        return ESP8266_OK;
    }
    
    // connection failed
    USER_LOG("❌ MQTT connection failed. Response: %s", esp_rx_buffer);
    mqtt_state = MQTT_ERROR;
    return result;
}

ESP8266_Status ESP_MQTT_Disconnect(void)
{
    ESP8266_Status result;
    
    USER_LOG("Disconnecting from MQTT broker...");
    
    result = ESP_DMA_SendCommand("AT+MQTTCLEAN=0\r\n", "OK", 5000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("MQTT disconnect failed");
        mqtt_state = MQTT_ERROR;
        return result;
    }
    
    mqtt_state = MQTT_DISCONNECTED;
    USER_LOG("MQTT disconnected");
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_Subscribe(const char *topic, MQTT_QoS qos)
{
    char cmd[256];
    ESP8266_Status result;
    
    if (mqtt_state != MQTT_CONNECTED)
    {
        DEBUG_LOG("MQTT not connected, cannot subscribe");
        return ESP8266_NOT_CONNECTED;
    }
    
    // Validate topic according to MQTT spec
    if (ESP_MQTT_ValidateTopic(topic, 1) != ESP8266_OK)
    {
        DEBUG_LOG("Invalid topic format for subscription: %s", topic);
        return ESP8266_ERROR;
    }
    
    // Validate QoS level
    if (qos > MQTT_QOS_2)
    {
        DEBUG_LOG("Invalid QoS level: %d", qos);
        return ESP8266_ERROR;
    }
    
    USER_LOG("Subscribing to MQTT topic: %s (QoS %d)", topic, qos);
    
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",%d\r\n", topic, qos);
    
    result = ESP_DMA_SendCommand(cmd, "OK", 5000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("MQTT subscribe failed for topic: %s", topic);
        return result;
    }
    
    USER_LOG("Successfully subscribed to: %s", topic);
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_Unsubscribe(const char *topic)
{
    char cmd[256];
    ESP8266_Status result;
    
    if (mqtt_state != MQTT_CONNECTED)
    {
        DEBUG_LOG("MQTT not connected, cannot unsubscribe");
        return ESP8266_NOT_CONNECTED;
    }
    
    USER_LOG("Unsubscribing from MQTT topic: %s", topic);
    
    snprintf(cmd, sizeof(cmd), "AT+MQTTUNSUB=0,\"%s\"\r\n", topic);
    
    result = ESP_DMA_SendCommand(cmd, "OK", 5000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("MQTT unsubscribe failed for topic: %s", topic);
        return result;
    }
    
    USER_LOG("Successfully unsubscribed from: %s", topic);
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_Publish(const char *topic, const char *message, MQTT_QoS qos, uint8_t retain)
{
    char cmd[1024];
    char escaped_message[512];
    ESP8266_Status result;
    
    if (mqtt_state != MQTT_CONNECTED)
    {
        DEBUG_LOG("MQTT not connected, cannot publish");
        return ESP8266_NOT_CONNECTED;
    }
    
    USER_LOG("Publishing to MQTT topic: %s", topic);
    DEBUG_LOG("Message: %s", message);
    
    // Check message length and simplify if too long
    if (strlen(message) > 100) {
        USER_LOG("Message too long (%d chars), using simple format", strlen(message));
        
        // Parse temperature, humidity, pressure from JSON message
        float temp = 0.0, hum = 0.0, press = 0.0;
        sscanf(message, "{\"temperature\":%f,\"humidity\":%f,\"pressure\":%f", &temp, &hum, &press);
        
        snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=0,\"%s\",\"T:%.1f H:%.1f P:%.1f\",%d,%d\r\n", 
                 topic, temp, hum, press, qos, retain);
    } else {
        // Escape quotes in JSON message for AT command
        int src_idx = 0, dst_idx = 0;
        while (message[src_idx] != '\0' && dst_idx < sizeof(escaped_message) - 2)
        {
            if (message[src_idx] == '"')
            {
                escaped_message[dst_idx++] = '\\';
                escaped_message[dst_idx++] = '"';
            }
            else
            {
                escaped_message[dst_idx++] = message[src_idx];
            }
            src_idx++;
        }
        escaped_message[dst_idx] = '\0';
        
        // Official ESP32 AT MQTT Publish Command with escaped message
        snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=0,\"%s\",\"%s\",%d,%d\r\n", 
                 topic, escaped_message, qos, retain);
    }
    
    result = ESP_DMA_SendCommand(cmd, "OK", 5000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("MQTT publish failed for topic: %s", topic);
        return result;
    }
    
    USER_LOG("Successfully published to: %s", topic);
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_CheckMessages(MQTT_Message *msg)
{
    uint8_t temp_buffer[1024];
    uint16_t received;
    
    if (!msg)
    {
        return ESP8266_ERROR;
    }
    
    // Clear message structure
    memset(msg, 0, sizeof(MQTT_Message));
    
    // Check for incoming data
    received = ESP_DMA_GetReceivedData(temp_buffer, sizeof(temp_buffer));
    
    if (received > 0)
    {
        temp_buffer[received] = '\0';
        char *buffer_str = (char*)temp_buffer;
        
        // Look for MQTT message pattern: +MQTTSUBRECV:0,"topic","message"
        char *mqtt_msg = strstr(buffer_str, "+MQTTSUBRECV:");
        if (mqtt_msg)
        {
            DEBUG_LOG("MQTT message received: %s", mqtt_msg);
            
            // Parse the message
            char *topic_start = strchr(mqtt_msg, '"');
            if (topic_start)
            {
                topic_start++; // Skip opening quote
                char *topic_end = strchr(topic_start, '"');
                if (topic_end)
                {
                    // Extract topic
                    int topic_len = topic_end - topic_start;
                    if (topic_len < MQTT_MAX_TOPIC_LEN)
                    {
                        strncpy(msg->topic, topic_start, topic_len);
                        msg->topic[topic_len] = '\0';
                        
                        // Find message part
                        char *msg_start = strchr(topic_end + 1, '"');
                        if (msg_start)
                        {
                            msg_start++; // Skip opening quote
                            char *msg_end = strchr(msg_start, '"');
                            if (msg_end)
                            {
                                // Extract message
                                int msg_len = msg_end - msg_start;
                                if (msg_len < MQTT_MAX_MESSAGE_LEN)
                                {
                                    strncpy(msg->message, msg_start, msg_len);
                                    msg->message[msg_len] = '\0';
                                    msg->message_len = msg_len;
                                    
                                    USER_LOG("MQTT Received - Topic: %s, Message: %s", 
                                             msg->topic, msg->message);
                                    return ESP8266_OK;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    return ESP8266_NO_RESPONSE;
}

MQTT_ConnectionState ESP_MQTT_GetState(void)
{
    return mqtt_state;
}

ESP8266_Status ESP_MQTT_ValidateTopic(const char *topic, uint8_t is_subscription)
{
    if (!topic || strlen(topic) == 0)
    {
        DEBUG_LOG("Topic cannot be empty");
        return ESP8266_ERROR;
    }
    
    if (strlen(topic) > MQTT_MAX_TOPIC_LEN - 1)
    {
        DEBUG_LOG("Topic too long: %d (max %d)", strlen(topic), MQTT_MAX_TOPIC_LEN - 1);
        return ESP8266_ERROR;
    }
    
    // Check for invalid characters (MQTT spec: UTF-8, no null characters)
    for (int i = 0; topic[i] != '\0'; i++)
    {
        // Check for null character
        if (topic[i] == '\0')
        {
            DEBUG_LOG("Topic contains null character");
            return ESP8266_ERROR;
        }
        
        // Check for control characters (0x00-0x1F, 0x7F-0x9F)
        if ((topic[i] >= 0x00 && topic[i] <= 0x1F) || 
            (topic[i] >= 0x7F && topic[i] <= 0x9F))
        {
            DEBUG_LOG("Topic contains control character: 0x%02X", topic[i]);
            return ESP8266_ERROR;
        }
    }
    
    // For publish topics, wildcards are not allowed
    if (!is_subscription)
    {
        if (strchr(topic, '+') || strchr(topic, '#'))
        {
            DEBUG_LOG("Wildcards not allowed in publish topics");
            return ESP8266_ERROR;
        }
    }
    else
    {
        // For subscription topics, validate wildcard usage
        char *plus_pos = strchr(topic, '+');
        char *hash_pos = strchr(topic, '#');
        
        // # wildcard must be at the end and preceded by /
        if (hash_pos)
        {
            if (hash_pos != topic + strlen(topic) - 1)
            {
                DEBUG_LOG("# wildcard must be at end of topic");
                return ESP8266_ERROR;
            }
            if (hash_pos != topic && *(hash_pos - 1) != '/')
            {
                DEBUG_LOG("# wildcard must be preceded by /");
                return ESP8266_ERROR;
            }
        }
        
        // + wildcard must be between / characters or at start/end
        if (plus_pos)
        {
            char *current = plus_pos;
            while (current)
            {
                if (current != topic && *(current - 1) != '/')
                {
                    DEBUG_LOG("+ wildcard must be preceded by /");
                    return ESP8266_ERROR;
                }
                if (*(current + 1) != '\0' && *(current + 1) != '/')
                {
                    DEBUG_LOG("+ wildcard must be followed by / or end");
                    return ESP8266_ERROR;
                }
                current = strchr(current + 1, '+');
            }
        }
    }
    
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_ValidateClientId(const char *client_id)
{
    if (!client_id)
    {
        DEBUG_LOG("Client ID cannot be null");
        return ESP8266_ERROR;
    }
    
    size_t len = strlen(client_id);
    
    // MQTT spec: Client ID can be empty (server assigns one)
    if (len == 0)
    {
        DEBUG_LOG("Empty client ID - server will assign one");
        return ESP8266_OK;
    }
    
    // Check length (MQTT spec allows up to 65535, but we limit to our buffer)
    if (len > MQTT_MAX_CLIENT_ID_LEN - 1)
    {
        DEBUG_LOG("Client ID too long: %d (max %d)", len, MQTT_MAX_CLIENT_ID_LEN - 1);
        return ESP8266_ERROR;
    }
    
    // Check for valid UTF-8 characters (simplified check)
    for (int i = 0; client_id[i] != '\0'; i++)
    {
        // Check for control characters
        if ((client_id[i] >= 0x00 && client_id[i] <= 0x1F) || 
            (client_id[i] >= 0x7F && client_id[i] <= 0x9F))
        {
            DEBUG_LOG("Client ID contains control character: 0x%02X", client_id[i]);
            return ESP8266_ERROR;
        }
    }
    
    return ESP8266_OK;
}

ESP8266_Status ESP_MQTT_Ping(void)
{
    ESP8266_Status result;
    
    if (mqtt_state != MQTT_CONNECTED)
    {
        DEBUG_LOG("MQTT not connected, cannot ping");
        return ESP8266_NOT_CONNECTED;
    }
    
    DEBUG_LOG("Sending MQTT ping");
    
    // ESP8266 AT firmware handles PINGREQ/PINGRESP automatically
    // We can verify connection by checking status
    result = ESP_DMA_SendCommand("AT+MQTTCONN?\r\n", "+MQTTCONN:0", 3000);
    
    if (result == ESP8266_OK)
    {
        DEBUG_LOG("MQTT ping successful");
        return ESP8266_OK;
    }
    else
    {
        DEBUG_LOG("MQTT ping failed - connection may be lost");
        mqtt_state = MQTT_ERROR;
        return ESP8266_ERROR;
    }
}

ESP8266_Status ESP_MQTT_CheckConnection(void)
{
    uint8_t temp_buffer[256];
    uint16_t received;
    ESP8266_Status result;
    
    // Check for connection status messages
    received = ESP_DMA_GetReceivedData(temp_buffer, sizeof(temp_buffer));
    
    if (received > 0)
    {
        temp_buffer[received] = '\0';
        char *buffer_str = (char*)temp_buffer;
        
        // Look for MQTT connection status
        if (strstr(buffer_str, "MQTT CONNECTED"))
        {
            return ESP8266_OK;
        }
        else if (strstr(buffer_str, "MQTT DISCONNECTED") || 
                 strstr(buffer_str, "MQTT: CONNECT REFUSED"))
        {
            return ESP8266_ERROR;
        }
    }
    
    // Check MQTT connection status using query command
    result = ESP_DMA_SendCommand("AT+MQTTCONN?\r\n", "+MQTTCONN:", 3000);
    if (result == ESP8266_OK) {
        // Parse the response to check connection status
        if (strstr(esp_rx_buffer, "+MQTTCONN:0,1")) {
            DEBUG_LOG("MQTT connection verified as active");
            return ESP8266_OK;
        } else if (strstr(esp_rx_buffer, "+MQTTCONN:0,0")) {
            DEBUG_LOG("MQTT connection verified as inactive");
            return ESP8266_ERROR;
        }
    }
    
    // If query fails, try ping as fallback
    return ESP_MQTT_Ping();
}


