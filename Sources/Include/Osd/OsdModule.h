#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include "Config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Uart.h"
#include "optional"
#include <vector>

// ---- 0xA0 - Configure UART ----
struct ConfigureUartRawCommand
{
    uint32_t baudRate : 22;
    uint32_t stopBits : 2;
    uint32_t parityBit : 2;
    uint32_t bitOrdering : 1;
    uint32_t unused : 5;
};
static_assert(sizeof(ConfigureUartRawCommand) == 4);

enum class StopBits
{
    HalfBit = 0,
    OneBit = 1,
    OneAndAHalfBits = 2,
    TwoBits = 3
};

enum class ParityBit
{
    Disabled = 0,
    Uneven = 1,
    Even = 2,
    NotUsed = 3
};

// ---- 0xA1 - Configure Output ----
struct ConfigureOutputRawCommand
{
    uint8_t outputMode : 2;
    uint8_t videoStandard : 1;
    uint8_t vSync : 1;
    uint8_t autoBlackLevel : 1;
    uint8_t unused : 3;
};
static_assert(sizeof(ConfigureOutputRawCommand) == 1);

enum class OutputMode
{
    Disabled = 0,
    VideoOnly = 1,
    TextOnly = 2,
    TextAndVideo = 3
};

enum class VideoStandard
{
    Ntsc = 0,
    Pal = 1
};

// ---- 0xA2 - Configure Line Formatting ----
struct ConfigureLineFormattingRawCommand
{
    uint8_t tabSize : 4;
    uint8_t wrapMethod : 2;
    uint8_t newLineBehaviour : 1;
    uint8_t unused : 1;
};
static_assert(sizeof(ConfigureLineFormattingRawCommand) == 1);

enum class WrapMethod
{
    WrapAtScreenEdge = 0,
    WrapWithoutBreakingWords = 1,
    Truncate = 2,
    NotUsed = 3
};

enum class NewLineBehaviour
{
    ContinueFromColumnZero = 0,
    ContinueFromCommandColumn = 1
};

// ---- 0xA3 - Configure Character Display ----
struct ConfigureCharacterDisplayRawCommand
{
    uint8_t background : 1;
    uint8_t invert : 1;
    uint8_t blink : 1;
    uint8_t unused : 5;
};
static_assert(sizeof(ConfigureCharacterDisplayRawCommand) == 1);

enum class CharacterBackground
{
    Transparent = 0,
    Gray = 1
};

// ---- 0xA9 - Set Row Brightness ----
struct SetRowBrightnessRawCommand
{
    uint8_t row : 4;
    uint8_t blackLevel : 2;
    uint8_t whiteLevel : 2;
};
static_assert(sizeof(SetRowBrightnessRawCommand) == 1);

enum class BlackLevel
{
    Percent0 = 0,
    Percent10 = 1,
    Percent20 = 2,
    Percent30 = 3
};

enum class WhiteLevel
{
    Percent120 = 0,
    Percent100 = 1,
    Percent90 = 2,
    Percent80 = 3
};

// ---- 0xAA - Configure Global Display Params ----
// Byte 0 (sharpness, 0-35) is passed as a plain value; byte 1 is bitpacked below.
struct ConfigureGlobalDisplayParamsRawCommand
{
    uint8_t backgroundBrightness : 3;
    uint8_t blinkingPeriod : 2;
    uint8_t blinkingDutyCycle : 2;
    uint8_t unused : 1;
};
static_assert(sizeof(ConfigureGlobalDisplayParamsRawCommand) == 1);

enum class BackgroundBrightness
{
    Percent0 = 0,
    Percent7 = 1,
    Percent14 = 2,
    Percent21 = 3,
    Percent28 = 4,
    Percent35 = 5,
    Percent42 = 6,
    Percent49 = 7
};

enum class BlinkingPeriod
{
    Frames2 = 0,
    Frames4 = 1,
    Frames6 = 2,
    Frames8 = 3
};

enum class BlinkingDutyCycle
{
    Ratio1To1 = 0,
    Ratio1To2 = 1,
    Ratio1To3 = 2,
    Ratio3To1 = 3
};

// ---- 0xAB - Get Input Signal Status (response byte) ----
enum class InputSignalStatus : uint8_t
{
    NoSignal = 0x20,
    Pal = 0x21,
    Ntsc = 0x22
};

// ---- 0xAC - Overwrite Character Image ----
// 12x18 px, 2 bits/px. Byte-level packing (4 px per byte) is an implementation
// detail handled in the .cpp; callers just work with a color grid.
enum class PixelColor : uint8_t
{
    Black = 0,
    White = 1,
    Background = 2
};

inline constexpr uint8_t CharacterBitmapWidthPx = 12;
inline constexpr uint8_t CharacterBitmapHeightPx = 18;

using CharacterBitmap = std::array<std::array<PixelColor, CharacterBitmapWidthPx>, CharacterBitmapHeightPx>;

// ---- 0xAD - Restore Default Settings ----
struct RestoreDefaultSettingsRawCommand
{
    uint8_t restoreOutputParams : 1;
    uint8_t restoreUartParams : 1;
    uint8_t restoreFont : 1;
    uint8_t restoreInitSequence : 1;
    uint8_t unused : 4;
};
static_assert(sizeof(RestoreDefaultSettingsRawCommand) == 1);

class IOsdResponseReader
{
public:
    virtual ~IOsdResponseReader() = default;
    virtual bool ReadBytes(std::span<uint8_t> buffer, uint32_t timeoutMs) = 0;
};

class OsdResponseReader : public IReader, public IOsdResponseReader
{
public:
    OsdResponseReader();
    ~OsdResponseReader() override;

    void OnByte(uint8_t byte) override;

    bool ReadBytes(std::span<uint8_t> buffer, uint32_t timeoutMs) override;

private:
    QueueHandle_t m_queue;
    StaticQueue_t m_queueBuffer{};
    uint8_t m_queueStorage[Config::OSD_RESPONSE_READER_QUEUE_SIZE];
};

class OsdModule
{
public:
    OsdModule(std::unique_ptr<IWriter> writer, std::shared_ptr<IOsdResponseReader> reader);

    // 0xA0
    void ConfigureUart(uint32_t baudRate, StopBits stopBits, ParityBit parityBit, bool isMsbFirst);

    // 0xA1
    void ConfigureOutput(OutputMode outputMode, VideoStandard videoStandard, bool isVSyncEnabled, bool isAutoBlackLevelEnabled);

    // 0xA2 - tabSizeInCharacters: 1-16
    void ConfigureLineFormatting(uint8_t tabSizeInCharacters, WrapMethod wrapMethod, NewLineBehaviour newLineBehaviour);

    // 0xA3
    void ConfigureCharacterDisplay(CharacterBackground background, bool isInverted, bool isBlinking);

    // 0xA4 - row: 0-11 (NTSC) / 0-15 (PAL), column: 0-29
    void DisplayCharacter(uint8_t row, uint8_t column, uint8_t characterCode);

    // 0xA5 - Cp1251 text, respects tab/newline formatting from 0xA2
    void DisplayLine(uint8_t row, uint8_t column, std::string_view text);

    // 0xA6 - Cp1251 text, \n and \t are treated as plain characters
    void DisplayUnformattedLine(uint8_t row, uint8_t column, std::string_view text);

    // 0xA7
    void ClearScreen();

    // 0xA8 - verticalOffsetPx: -15..16, horizontalOffsetPx: -32..31
    void SetOffset(int8_t verticalOffsetPx, int8_t horizontalOffsetPx);

    // 0xA9
    void SetRowBrightness(uint8_t row, BlackLevel blackLevel, WhiteLevel whiteLevel);

    // 0xAA - sharpness: 0 (max blur) .. 35 (max sharp)
    void ConfigureGlobalDisplayParams(uint8_t sharpness, BackgroundBrightness backgroundBrightness,
                                       BlinkingPeriod blinkingPeriod, BlinkingDutyCycle blinkingDutyCycle);

    // 0xAB
    std::optional<InputSignalStatus> GetInputSignalStatus(uint32_t timeoutMs = 100);

    // 0xAC - text output is unavailable on the module while this executes
    void OverwriteCharacterImage(uint8_t characterCode, const CharacterBitmap& bitmap);

    // 0xAD
    void RestoreDefaultSettings(bool restoreOutputParams, bool restoreUartParams, bool restoreFont, bool restoreInitSequence);

    // 0xAE - returns sequence number (0-127) assigned by the module
    std::optional<uint8_t> SaveCommandSequence(std::span<const uint8_t> commandSequence, uint32_t timeoutMs = 100);

    // 0xAF
    void ExecuteCommandSequence(uint8_t sequenceNumber);

    // 0xB0
    void DeleteCommandSequence(uint8_t sequenceNumber);

    // 0xB1
    void ClearCommandBuffer();

    // 0xB2
    void Delay(uint8_t delayMs);

private:
    void WriteCommand(uint8_t commandId, std::span<const uint8_t> argument);

    std::unique_ptr<IWriter> m_writer;
    std::shared_ptr<IOsdResponseReader> m_reader;
};
