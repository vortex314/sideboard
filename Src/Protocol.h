#include <stdint.h>
#include <cstring>
#include <errno.h>

class Fcs
{
    uint16_t _fcs;

public:
    Fcs() { _fcs = 0xFFFF; }
    bool hasSpace(int size = 1) { return true; };
    bool write(uint8_t b);
    uint16_t intermediate() const { return _fcs; };
    uint16_t result() const { return ~_fcs; }
    void clear() { _fcs = 0xFFFF; }
};

#define PPP_FCS_SIZE 2

#define PPP_MASK_CHAR 0x20
#define PPP_ESC_CHAR 0x7D
#define PPP_FLAG_CHAR 0x7E

typedef enum
{
    MT_INVALID = -1,
    MT_UNSIGNED = 0,
    MT_NEGATIVE = 1,
    MT_BYTES = 2,
    MT_TEXT = 3,
    MT_ARRAY = 4,
    MT_MAP = 5,
    MT_TAG = 6,
    MT_PRIMITIVE = 7,
    MT_INDEFINITE = 8,
    MT_RAW = 9
} MajorType;

typedef enum CborType
{
    CborIntegerType = 0x00,
    CborByteStringType = 0x40,
    CborTextStringType = 0x60,
    CborArrayType = 0x80,
    CborMapType = 0xa0,
    CborTagType = 0xc0,
    CborSimpleType = 0xe0,
    CborBooleanType = 0xf5,
    CborNullType = 0xf6,
    CborUndefinedType = 0xf7,
    CborHalfFloatType = 0xf9,
    CborFloatType = 0xfa,
    CborDoubleType = 0xfb,

    CborInvalidType = 0xff /* equivalent to the break byte, so it will never be used */
} CborType;

typedef enum
{
    DIRECT = -1,
    ONE_BYTE = 24,
    TWO_BYTES = 25,
    FOUR_BYTES = 26,
    EIGHT_BYTES = 27,
    RESERVED = 28,
    INDEFINITE = 31
} AdditionalInformation;

class ProtocolEncoder
{
public:
    ProtocolEncoder(uint32_t);
    bool ok();
    ProtocolEncoder &start();
    ProtocolEncoder &end();
    ProtocolEncoder &encode(int);
    ProtocolEncoder &encode(unsigned int);
    ProtocolEncoder &encode(long);
    ProtocolEncoder &encode(unsigned long);
    ProtocolEncoder &encode(long long);
    ProtocolEncoder &encode(unsigned long long);
    ProtocolEncoder &encode(float);
    ProtocolEncoder &encode(double);
    ProtocolEncoder &encode(const char *);
    ProtocolEncoder &encode(bool);
    ProtocolEncoder &encodeArrayStart();
    ProtocolEncoder &encodeArrayEnd();
    ProtocolEncoder &encodeMapStart();
    ProtocolEncoder &encodeMapEnd();

    inline uint8_t *buffer() { return _buffer; }
    inline uint32_t size() { return _index; }

private:
    uint8_t *_buffer;
    uint32_t _capacity;
    uint32_t _index;
    Fcs _fcs;
    int _error;
    void addByte(uint8_t value);
    void addEscaped(uint8_t);
    void addEscaped(uint8_t *, uint32_t);
    void encode_type_and_length(MajorType majorType, uint64_t length);
};

class ProtocolDecoder;
#include <string>
typedef void (*ProtocolHandler)(ProtocolDecoder *);

class ProtocolDecoder
{
public:
    ProtocolDecoder(uint32_t);
    inline bool ok() { return _error == 0; };
    void deframe(uint8_t *, uint32_t);
    MajorType majorType();
    void reset();
    bool checkCrc();
    ProtocolDecoder& decodeArrayStart();
    ProtocolDecoder& decodeArrayEnd();
    ProtocolDecoder& decodeMapStart();
    ProtocolDecoder& decodeMapEnd();
    ProtocolDecoder& get(int &);
    ProtocolDecoder& get(unsigned int &);
    ProtocolDecoder& get(long &);
    ProtocolDecoder& get(unsigned long &);
    ProtocolDecoder& get(long long &);
    ProtocolDecoder& get(unsigned long long &);
    ProtocolDecoder& get(float &);
    ProtocolDecoder& get(double &);
    ProtocolDecoder& get(char *,uint32_t);
    ProtocolDecoder& decode(std::string&);
    void addByte(uint8_t);
    uint8_t *buffer() { return _buffer; }
    uint32_t size() { return _index; }
    int error() { return _error; }
    ProtocolDecoder& rewind() ;

private:
    uint8_t *_buffer;
    uint32_t _capacity;
    uint32_t _index;
    uint32_t _readPtr;
    Fcs _fcs;
    MajorType _type;
    int _error;
    uint8_t read();
    void advance(uint32_t);
};
