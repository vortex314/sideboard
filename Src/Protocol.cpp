#include <Protocol.h>

extern "C"
{
#include "defines.h"
#include "gd32f1x0_usart.h"
#include "defines.h"
#include "setup.h"
    extern MPU_Data mpu;
    extern uint32_t main_loop_counter;
    extern FlagStatus sensor1, sensor2;
    extern ErrStatus mpuStatus;
}

#define INPUT_SIZE 256
#define OUTPUT_SIZE 256

ProtocolEncoder *encoder;
ProtocolDecoder *decoder;

extern "C" void protocol_init()
{
    encoder = new ProtocolEncoder(OUTPUT_SIZE);
    decoder = new ProtocolDecoder(INPUT_SIZE);
    usart_Tx_DMA_config(USART_MAIN, encoder->buffer(), encoder->size());
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

extern "C" void protocol_loop()
{
    if ((main_loop_counter % 100) == 0 && dma_transfer_number_get(USART1_TX_DMA_CH) == 0)
    {
        encoder->start().encodeArrayStart().encode("publish").encodeMapStart();
        encoder->encode("src").encode("sideboard");
        encoder->encode("mpu/temp").encode(mpu.temp);
        encoder->encode("mpu/status").encode(mpuStatus);
        encoder->encode("sensor/left").encode(sensor1);
        encoder->encode("sensor/right").encode(sensor2);
        encoder->encode("acc/x").encode(mpu.accel.x).encode("acc/y").encode(mpu.accel.y).encode("acc/z").encode(mpu.accel.z);
        encoder->encode("gyro/x").encode(mpu.gyro.x).encode("gyro/y").encode(mpu.gyro.y).encode("gyro/z").encode(mpu.gyro.z);
        encoder->encode("quat/w").encode(mpu.quat.w).encode("quat/x").encode(mpu.quat.x).encode("quat/y").encode(mpu.quat.y).encode("quat/z").encode(mpu.quat.z);
        encoder->encode("euler/roll").encode(mpu.euler.roll).encode("euler/pitch").encode(mpu.euler.pitch).encode("euler/yaw").encode(mpu.euler.yaw);
        encoder->encodeMapEnd();
        encoder->encodeArrayEnd().end();
        usartSendDMA(encoder->buffer(), encoder->size());
    }
}
//==============================================================================
static const uint16_t fcsTable[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48,
    0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108,
    0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB,
    0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
    0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E,
    0xFAE7, 0xC87C, 0xD9F5, 0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E,
    0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD,
    0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285,
    0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44,
    0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72, 0x6306, 0x728F, 0x4014,
    0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
    0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3,
    0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862,
    0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E,
    0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1,
    0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483,
    0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50,
    0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
    0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7,
    0x6E6E, 0x5CF5, 0x4D7C, 0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1,
    0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72,
    0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E,
    0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF,
    0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9, 0xF78F, 0xE606, 0xD49D,
    0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
    0x3DE3, 0x2C6A, 0x1EF1, 0x0F78};

bool Fcs::write(uint8_t b)
{
    _fcs = (_fcs >> 8) ^ fcsTable[(_fcs & 0xFF) ^ b];
    return true;
}
//=============================================================================
ProtocolEncoder::ProtocolEncoder(uint32_t size)
{
    auto pui = new uint32_t[(size / 4) + 1]; // allocate space for the array in 4 byte words
    _buffer = (uint8_t *)pui;
    _capacity = ((size / 4) + 1) * 4;
    _index = 0;
}

bool ProtocolEncoder::ok()
{
    return _error == 0;
}

ProtocolEncoder &ProtocolEncoder::start()
{
    _index = 0;
    _error = 0;
    _fcs.clear();
    addByte(PPP_FLAG_CHAR);
    return *this;
}

ProtocolEncoder &ProtocolEncoder::end()
{
    uint16_t fcs = _fcs.result(); // addEscaped influences the result
    addEscaped(fcs & 0xFF);       // LSB first
    addEscaped(fcs >> 8);
    addByte(PPP_FLAG_CHAR);
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encode(int64_t value)
{
    /* adapted from code in RFC 7049 appendix C (pseudocode) */
    uint64_t ui;                                                   /* extend sign to whole length */
    MajorType majorType = (value < 0) ? MT_NEGATIVE : MT_UNSIGNED; /* extract major type */
    if (majorType == MT_NEGATIVE)
        ui = value ^ 0xFFFFFFFFFFFFFFFF; /* remove sign bit */
    else
        ui = value;
    encode_type_and_length(majorType, ui);
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encode(int value)
{
    return encode((int64_t)value);
}

ProtocolEncoder &ProtocolEncoder::encode(int32_t value)
{
    return encode((int64_t)value);
}

ProtocolEncoder &ProtocolEncoder::encode(const char *value)
{
    encode_type_and_length(MT_TEXT, strlen(value));
    addEscaped((uint8_t *)value, strlen(value));
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encode(bool b)
{
    addEscaped((MT_PRIMITIVE << 5) + 20 + (b ? 1 : 0));
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encode(float d)
{
    addEscaped(CborFloatType);
    uint8_t *pb = (uint8_t *)&d;
    for (int i = 0; i < 4; i++)
        addEscaped(*(pb + 3 - i));
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encode(double d)
{
    addEscaped(CborDoubleType);
    uint8_t *pb = (uint8_t *)&d;
    for (int i = 0; i < 8; i++)
        addEscaped(*(pb + 7 - i));
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encodeArrayStart()
{
    addEscaped((MT_ARRAY << 5) + 31);
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encodeArrayEnd()
{
    addEscaped((MT_PRIMITIVE << 5) + 31);
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encodeMapStart()
{
    addEscaped((MT_MAP << 5) + 31);
    return *this;
}

ProtocolEncoder &ProtocolEncoder::encodeMapEnd()
{
    addEscaped((MT_PRIMITIVE << 5) + 31);
    return *this;
}

void ProtocolEncoder::addEscaped(uint8_t *buffer, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
        addEscaped(buffer[i]);
}

void ProtocolEncoder::addEscaped(uint8_t value)
{
    _fcs.write(value);
    if (value == PPP_ESC_CHAR || value == PPP_FLAG_CHAR)
    { // byte stuffing
        addByte(PPP_ESC_CHAR);
        addByte(value ^ PPP_MASK_CHAR);
    }
    else
    {
        addByte(value);
    }
}
void ProtocolEncoder::addByte(uint8_t value)
{
    if (_index + 1 > _capacity)
    {
        _error = ENOMEM;
        return;
    }
    _buffer[_index++] = value;
}

void ProtocolEncoder::encode_type_and_length(MajorType majorType, uint64_t length)
{
    int symbol = majorType << 5;
    if (length <= 23L)
    {
        addEscaped((uint8_t)(symbol | length));
    }
    else if (length <= 255L)
    {
        symbol |= ONE_BYTE;
        addEscaped((uint8_t)symbol);
        addEscaped((uint8_t)length);
    }
    else if (length <= 65535L)
    {
        symbol |= TWO_BYTES;
        addEscaped((uint8_t)symbol);
        addEscaped((uint8_t)(length >> 8));
        addEscaped((uint8_t)(length & 0xFF));
    }
    else if (length <= 4294967295L)
    {
        symbol |= FOUR_BYTES;
        addEscaped((uint8_t)symbol);
        addEscaped((uint8_t)((length >> 24) & 0xFF));
        addEscaped((uint8_t)((length >> 16) & 0xFF));
        addEscaped((uint8_t)((length >> 8) & 0xFF));
        addEscaped((uint8_t)(length & 0xFF));
    }
    else
    {
        symbol |= EIGHT_BYTES;
        addEscaped((uint8_t)symbol);
        addEscaped((uint8_t)((length >> 56) & 0xFF));
        addEscaped((uint8_t)((length >> 48) & 0xFF));
        addEscaped((uint8_t)((length >> 40) & 0xFF));
        addEscaped((uint8_t)((length >> 32) & 0xFF));
        addEscaped((uint8_t)((length >> 24) & 0xFF));
        addEscaped((uint8_t)((length >> 16) & 0xFF));
        addEscaped((uint8_t)((length >> 8) & 0xFF));
        addEscaped((uint8_t)(length & 0xFF));
    }
}

//===============================================================

ProtocolDecoder::ProtocolDecoder(uint32_t size)
{
    _buffer = new uint8_t[size];
    _capacity = size;
    _index = 0;
}