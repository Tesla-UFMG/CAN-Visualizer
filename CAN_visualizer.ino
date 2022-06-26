/**
 * @file CAN_visualizer.ino
 * @author Felipe Telles (felipe.melo.telles@gmail.com)
 * @brief Sends and Receives CAN messages with lots of configurable parameters.
 * CAN_visualizer[...]() and default arduino functions are designed to be edited by the
 * user. The others function should not be edited.
 * Uses coryjfowler's MCP2515 library, available from arduino library manager (Tools ->
 * Manage Libraries...) and from github (https://github.com/coryjfowler/MCP_CAN_lib)
 * @date 26-06-2022
 *
 */

#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <stdbool.h>
#include <stdint.h>

#define MCP2515_CS_PIN  7
#define MCP2515_INT_PIN 8

#define CAN_MESSAGE_SIZE  8
#define CAN_MESSAGE_FRAME 0

#define CAN_BAUDRATE  CAN_500KBPS
#define MCP2515_CLOCK MCP_8MHZ

#define DELAY_MESSAGE_TRANSMIT 100

/**
 * @brief Creates CAN instance
 *
 * @return MCP_CAN
 */
MCP_CAN CAN(MCP2515_CS_PIN);

/**
 * @brief creates timer instance
 *
 */
uint16_t send_timer = 0;

uint16_t id = 0;
char print_string[128];

void setup() {
    Serial.begin(115200);

    while (CAN_init(CAN_BAUDRATE, MCP2515_CLOCK) == false) {
        Serial.println("Error Initializing MCP2515");
        delay(500);
    }
    Serial.println("MCP2515 Initialized Successfully!");
}

void loop() {
    if (!digitalRead(MCP2515_INT_PIN)) {
        CAN_visualizer_receive_message();
    }

    if (timer_wait(send_timer, DELAY_MESSAGE_TRANSMIT)) {
        CAN_visualizer_send_message();
        timer_restart(&send_timer);
    }
}

void CAN_visualizer_receive_message() {
    uint16_t data_RX[4];
    uint16_t id_RX;
    CAN_receive_message(data_RX, &id_RX);

    sprintf(print_string, "Id: %u", id_RX);
    Serial.print(print_string);
    sprintf(print_string, "\tWord1: \t%u \tWord2: %u \tWord3: %u \tWord4: %u", data_RX[0],
            data_RX[1], data_RX[2], data_RX[3]);
    Serial.println(print_string);
}

void CAN_visualizer_send_message() {
    if (id >= 400) {
        id = 1;
    } else {
        id++;
    }

    const uint16_t timer      = (uint16_t)millis();
    const uint16_t canMsg1[4] = {timer, 0, 0xFFFF, id};

    if (CAN_send_message(canMsg1, id)) {
        Serial.print("Message sent successfully! Id = ");
        Serial.println(id);
    } else {
        Serial.print("Error sending message! Id = ");
        Serial.println(id);
    }
}

/**
 * @brief Receives CAN message.
 *
 * @param data_to_receive Array that received data will be stored.
 * @param id Address to variable where id will be stored.
 */
void CAN_receive_message(uint16_t* data_to_receive, uint16_t* id) {
    unsigned char len = 8, flag_frame = 0;
    CAN.readMsgBuf((uint32_t*)id, &flag_frame, &len, (uint8_t*)data_to_receive);
}

/**
 * @brief Sends CAN message.
 *
 * @param data_to_send Array with the data that will be sent.
 * @param id_to_send Message ID.
 * @return true Message sent successfully.
 * @return false Message not sent successfully.
 */
bool CAN_send_message(uint16_t* data_to_send, uint16_t id_to_send) {
    uint8_t status = CAN.sendMsgBuf(id_to_send, CAN_MESSAGE_FRAME, CAN_MESSAGE_SIZE,
                                    (uint8_t*)data_to_send);
    if (status == CAN_OK) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Initialize CAN bus at the desired baudrate. Also sets normal CAN mode
 * and sets interruption pin.
 *
 * @param CAN_baudrate Desired baudrate, should use the following defines:
 * CAN_5KBPS, CAN_10KBPS, CAN_20KBPS, CAN_50KBPS, CAN_100KBPS, CAN_125KBPS,
 * CAN_250KBPS, CAN_5000KBPS, CAN_1000KBPS.
 * @param CAN_mcp_clock MCP2515 clock crystal speed, should use the following
 * defines: MCP_8MHZ, MCP_16MHZ.
 * @return true Successful initialization.
 * @return false Unsuccessful initialization.
 */
bool CAN_init(uint8_t CAN_baudrate, uint8_t CAN_mcp_clock) {
    uint8_t status = CAN.begin(MCP_ANY, CAN_baudrate, CAN_mcp_clock);
    if (status == CAN_OK) {
        CAN.setMode(MCP_NORMAL);
        pinMode(MCP2515_INT_PIN, INPUT);
        return true;
    }
    return false;
}

/**
 * @brief Waits a specific timer to be elapsed by a desired amount. Allows
 * multiple timers by sending a different timer_start.
 *
 * @param timer_start Timer specifier.
 * @param delay Amount desired to wait, max is 65535, which is equal to
 * approximately 65 seconds. Values higher than that will not work as expected!
 * @return true The desired time has elapsed.
 * @return false The desired time has not been elapsed.
 */
bool timer_wait(uint16_t timer_start, uint16_t delay) {
    const uint16_t current_time = (uint16_t)millis();
    if ((current_time - timer_start) >= delay) {
        return true;
    }
    return false;
}

/**
 * @brief Restart timer to current millis value.
 *
 * @param timer_to_restart
 */
void timer_restart(uint16_t* timer_to_restart) {
    *timer_to_restart = (uint16_t)millis();
}
