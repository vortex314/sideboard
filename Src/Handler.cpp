
#include <Protocol.h>

extern "C"
{
#include "defines.h"
#include "gd32f1x0_usart.h"
#include "defines.h"
#include "setup.h"
#include "mpu6050.h"
#include "i2c_it.h"
#include "util.h"
#include <systick.h>

    extern MPU_Data mpu;
    extern uint32_t main_loop_counter;
    extern FlagStatus sensor1, sensor2;
    extern ErrStatus mpuStatus;
}

void usartSendDMA(uint8_t *buffer, uint32_t size);
void usartSendDMA(Bytes *bs);
void usartSend(uint8_t *buffer, uint32_t size);
void handleMessage(ProtocolDecoder *decoder);

#define INPUT_SIZE 256
#define OUTPUT_SIZE 256

ProtocolEncoder *encoder;
ProtocolDecoder *decoder;
bool connected = false;
uint32_t lastLoopback = 0;

struct
{
    uint32_t port;
    uint32_t pin;
} ledTable[] = {{LED1_GPIO_Port, LED1_Pin},
                {LED2_GPIO_Port, LED2_Pin},
                {LED3_GPIO_Port, LED3_Pin},
                {LED4_GPIO_Port, LED4_Pin},
                {LED5_GPIO_Port, LED5_Pin}};

extern "C" void protocol_init()
{
    encoder = new ProtocolEncoder(OUTPUT_SIZE);
    decoder = new ProtocolDecoder(INPUT_SIZE);
    usart_Tx_DMA_config(USART_MAIN, encoder->data(), encoder->size());
}

extern "C" void protocol_loop()
{
    *encoder << Start;
    if (connected && (main_loop_counter % 1000) == 0 && dma_transfer_number_get(USART1_TX_DMA_CH) == 0)
    {
        *encoder << '[' << "pub"
                 << "src/sideboard/";
        *encoder << '{';
        *encoder << "mpu/temp" << mpu.temp;
        *encoder << "system/alive" << true;
        *encoder << "mpu/temp" << mpu.temp;
        *encoder << "mpu/status" << mpuStatus;
        *encoder << "sensor/left" << sensor1;
        *encoder << "sensor/right" << sensor2;
        *encoder << "acc/x" << mpu.accel.x;
        *encoder << "acc/y" << mpu.accel.y;
        *encoder << "acc/z" << mpu.accel.z;
        *encoder << "gyro/x" << mpu.gyro.x;
        *encoder << "gyro/y" << mpu.gyro.y;
        *encoder << "gyro/z" << mpu.gyro.z;
        *encoder << "quat/w" << mpu.quat.w;
        *encoder << "quat/x" << mpu.quat.x;
        *encoder << "quat/y" << mpu.quat.y;
        *encoder << "quat/z" << mpu.quat.z;
        *encoder << "euler/roll" << mpu.euler.roll;
        *encoder << "euler/pitch" << mpu.euler.pitch;
        *encoder << "euler/yaw" << mpu.euler.yaw;
        *encoder << '}';
        *encoder << ']' << End;
        usartSendDMA(encoder);
        if (mpu.euler.yaw == 0)
        {
            i2c_config(); // I2C config
            i2c_nvic_config();
            mpu_config();
        }
    }
    if (!connected && (main_loop_counter % 1000) == 0 && dma_transfer_number_get(USART1_TX_DMA_CH) == 0)
    {
        *encoder << Start << '[' << "sub"
                 << "dst/sideboard/*" << ']' << End;
        usartSendDMA(encoder);
    }
    if ((main_loop_counter % 1500) == 0 && dma_transfer_number_get(USART1_TX_DMA_CH) == 0)
    {
        uint32_t msec;
        get_tick_count_ms(&msec);
        *encoder << Start << '[' << "pub"
                 << "dst/sideboard/system/loopback" << msec << ']' << End;
        usartSendDMA(encoder);
    }
    uint32_t now;
    get_tick_count_ms(&now);
    if (now - lastLoopback > 3000)
    {
        connected = false;
    }
}

extern "C" void protocol_handle(uint8_t *buffer, uint32_t size)
{
    if (decoder == 0)
        return;
    for (uint32_t i = 0; i < size; i++)
    {
        auto b = buffer[i];
        if (b == PPP_FLAG_CHAR)
        {
            if (decoder->size() > 2 && decoder->ok() && decoder->checkCrc())
            {
                handleMessage(decoder);
            }
            decoder->reset();
        }
        else
        {
            decoder->addUnEscaped(b);
        }
    }
}
void handleMessage(ProtocolDecoder *decoder)
{
    std::string cmd;
    std::string topic;
    if (decoder->rewind().readArrayStart().read(cmd).read(topic).ok() && dma_transfer_number_get(USART1_TX_DMA_CH) == 0)
    {
        if (cmd == "pub")
        {
            if (topic == "dst/sideboard/system/loopback" && (decoder->peek().is_int() || decoder->peek().is_uint()))
            {
                uint32_t msec = 0;
                uint32_t now;
                get_tick_count_ms(&now);
                decoder->read(msec);

                *encoder << Start << '[' << "pub"
                         << "src/sideboard/system/latency" << (now - msec) * 1000 << ']' << End;
                usartSendDMA(encoder);
                connected = true;
                get_tick_count_ms(&lastLoopback);
                return;
            }

            if (topic.find("dst/sideboard/led/") != std::string::npos)
            {
                std::string op;
                if (decoder->read(op).ok())
                {
                    // extract led idx
                    char lastChar = topic.back();
                    int index = lastChar - '1';
                    if (index >= 0 && index < 5)
                    {
                        if (op == "on")
                        {
                            gpio_bit_set(ledTable[index].port, ledTable[index].pin);
                        }
                        else if (op == "off")
                        {
                            gpio_bit_reset(ledTable[index].port, ledTable[index].pin);
                        }
                        else if (op == "toggle")
                        {
                            toggle_led(ledTable[index].port, ledTable[index].pin);
                        }
                    }
                }
                return;
            }
            if (decoder->peek().is_float())
            {
                float value;
                if (decoder->read(value).ok())
                {
                    *encoder << Start << '[' << "pub"
                             << "testTopic" << value << ']' << End;
                    usartSendDMA(encoder);
                };
            }
            else
            {
                *encoder << Start << '[' << "pubRcv"
                         << "testTopic"
                         << "ok" << ']' << End;
                usartSendDMA(encoder);
            }
        }
    }
}

void usartSendDMA(Bytes *bs)
{
    usartSendDMA(bs->data(), bs->size());
}

void usartSendDMA(uint8_t *buffer, uint32_t size)
{
    if (dma_transfer_number_get(USART1_TX_DMA_CH) == 0)
    { // Check if DMA channel counter is 0 (meaning all data has been transferred)

        dma_channel_disable(USART1_TX_DMA_CH);
        DMA_CHCNT(USART1_TX_DMA_CH) = size;
        DMA_CHMADDR(USART1_TX_DMA_CH) = (uint32_t)buffer;
        dma_channel_enable(USART1_TX_DMA_CH);
    }
}

void usartSend(uint8_t *buffer, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        usart_data_transmit(USART_MAIN, (uint8_t)buffer[i]);
        while (RESET == usart_flag_get(USART_MAIN, USART_FLAG_TBE))
            ;
    }
}