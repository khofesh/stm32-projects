/**
 * https://controllerstech.com/stm32-esp8266-wifi-ip/
 */

#include "esp8266_stm32.h"

ESP8266_ConnectionState ESP_ConnState = ESP8266_DISCONNECTED; // Default state

// DMA buffers and variables
uint8_t esp_dma_rx_buffer[ESP_DMA_RX_BUFFER_SIZE];
uint8_t esp_dma_tx_buffer[ESP_DMA_TX_BUFFER_SIZE];
volatile uint16_t esp_dma_rx_head = 0;
volatile uint16_t esp_dma_rx_tail = 0;

// Legacy buffer
static char esp_rx_buffer[2048];

static ESP8266_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len);
static ESP8266_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout);

void ESP_DMA_Init(void)
{
    // Initialize DMA buffers
    memset(esp_dma_rx_buffer, 0, ESP_DMA_RX_BUFFER_SIZE);
    memset(esp_dma_tx_buffer, 0, ESP_DMA_TX_BUFFER_SIZE);
    esp_dma_rx_head = 0;
    esp_dma_rx_tail = 0;

    DEBUG_LOG("DMA buffers initialized for UART4");
}

void ESP_DMA_StartReceive(void)
{
    // Start DMA reception in circular mode
    HAL_UART_Receive_DMA(&ESP_UART, esp_dma_rx_buffer, ESP_DMA_RX_BUFFER_SIZE);
    DEBUG_LOG("DMA reception started for UART4");
}

uint16_t ESP_DMA_GetReceivedData(uint8_t *buffer, uint16_t max_len)
{
    uint16_t dma_head = ESP_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    uint16_t data_len = 0;

    if (dma_head != esp_dma_rx_tail)
    {
        if (dma_head > esp_dma_rx_tail)
        {
            data_len = dma_head - esp_dma_rx_tail;
            if (data_len > max_len)
                data_len = max_len;
            memcpy(buffer, &esp_dma_rx_buffer[esp_dma_rx_tail], data_len);
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
        DEBUG_LOG("DMA sending: %s", cmd);

        // Copy command to DMA TX buffer
        if (cmd_len > ESP_DMA_TX_BUFFER_SIZE)
            cmd_len = ESP_DMA_TX_BUFFER_SIZE;
        memcpy(esp_dma_tx_buffer, cmd, cmd_len);

        if (HAL_UART_Transmit_DMA(&ESP_UART, esp_dma_tx_buffer, cmd_len) != HAL_OK)
        {
            return ESP8266_ERROR;
        }

        // Wait for transmission to complete
        while (HAL_UART_GetState(&ESP_UART) == HAL_UART_STATE_BUSY_TX)
        {
            if ((HAL_GetTick() - tickstart) > timeout)
            {
                return ESP8266_TIMEOUT;
            }
        }
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

ESP8266_Status ESP_Init(void)
{
    ESP8266_Status res;
    USER_LOG("Initializing ESP8266 with DMA...");

    // Initialize DMA for UART4
    ESP_DMA_Init();
    ESP_DMA_StartReceive();

    HAL_Delay(1000);

    //	res = ESP_DMA_SendCommand("AT+RST\r\n", "OK", 2000);
    //    if (res != ESP8266_OK){
    //    	DEBUG_LOG("Failed to Reset ESP8266...");
    //    	return res;
    //    }
    //
    //    USER_LOG("Waiting 5 Seconds for Reset to Complete...");
    //    HAL_Delay(5000);  // wait for reset to complete

    res = ESP_DMA_SendCommand("AT\r\n", "OK", 2000);
    if (res != ESP8266_OK)
    {
        DEBUG_LOG("ESP8266 Not Responding...");
        return res;
    }

    res = ESP_DMA_SendCommand("ATE0\r\n", "OK", 2000); // Disable echo
    if (res != ESP8266_OK)
    {
        DEBUG_LOG("Disable echo Command Failed...");
        return res;
    }
    USER_LOG("ESP8266 Initialized Successfully with DMA...");
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

static ESP8266_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout)
{
    return ESP_DMA_SendCommand(cmd, ack, timeout);
}
