#include "OsdModule.h"
#include <algorithm>
#include <array>
#include <cassert>

OsdResponseReader::OsdResponseReader()
{
    m_streamBuffer = xStreamBufferCreateStatic(
        Config::OSD_RESPONSE_READER_BUFFER_SIZE, 1, m_streamBufferStorage, &m_streamBufferBuffer);
    configASSERT(m_streamBuffer != nullptr);
}

OsdResponseReader::~OsdResponseReader()
{
    vStreamBufferDelete(m_streamBuffer);
}

void OsdResponseReader::OnChunk(std::span<const uint8_t> chunk)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xStreamBufferSendFromISR(m_streamBuffer, chunk.data(), chunk.size(), &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

bool OsdResponseReader::ReadBytes(std::span<uint8_t> buffer, uint32_t timeoutMs)
{
    size_t bytesRead = 0;
    while (bytesRead < buffer.size())
    {
        const size_t received = xStreamBufferReceive(
            m_streamBuffer,
            buffer.data() + bytesRead,
            buffer.size() - bytesRead,
            pdMS_TO_TICKS(timeoutMs));
        if (received == 0)
            return false;
        bytesRead += received;
    }
    return true;
}

OsdModule::OsdModule(IWriter& writer, IOsdResponseReader& reader)
    : m_writer(writer)
    , m_reader(reader)
{
}

void OsdModule::WriteCommand(uint8_t commandId, std::span<const uint8_t> argument)
{
    m_writer.Write({&commandId, 1});
    if (!argument.empty())
    {
        m_writer.Write(argument);
    }
}

void OsdModule::ConfigureUart(uint32_t baudRate, StopBits stopBits, ParityBit parityBit, bool isMsbFirst)
{
    ConfigureUartRawCommand command{};
    command.baudRate = baudRate;
    command.bitOrdering = isMsbFirst ? 1 : 0;
    command.parityBit = static_cast<uint32_t>(parityBit);
    command.stopBits = static_cast<uint32_t>(stopBits);
    WriteCommand(0xA0, {reinterpret_cast<uint8_t*>(&command), sizeof(command)});
}

void OsdModule::ConfigureOutput(OutputMode outputMode, VideoStandard videoStandard, bool isVSyncEnabled, bool isAutoBlackLevelEnabled)
{
    ConfigureOutputRawCommand command{};
    command.outputMode = static_cast<uint8_t>(outputMode);
    command.videoStandard = static_cast<uint8_t>(videoStandard);
    command.vSync = isVSyncEnabled ? 1 : 0;
    command.autoBlackLevel = isAutoBlackLevelEnabled ? 1 : 0;
    WriteCommand(0xA1, {reinterpret_cast<uint8_t*>(&command), sizeof(command)});
}

void OsdModule::ConfigureLineFormatting(uint8_t tabSizeInCharacters, WrapMethod wrapMethod, NewLineBehaviour newLineBehaviour)
{
    assert(tabSizeInCharacters >= 1 && tabSizeInCharacters <= 16);

    ConfigureLineFormattingRawCommand command{};
    command.tabSize = static_cast<uint8_t>(std::clamp<uint8_t>(tabSizeInCharacters, 1, 16) - 1);
    command.wrapMethod = static_cast<uint8_t>(wrapMethod);
    command.newLineBehaviour = static_cast<uint8_t>(newLineBehaviour);
    WriteCommand(0xA2, {reinterpret_cast<uint8_t*>(&command), sizeof(command)});
}

void OsdModule::ConfigureCharacterDisplay(CharacterBackground background, bool isInverted, bool isBlinking)
{
    ConfigureCharacterDisplayRawCommand command{};
    command.background = static_cast<uint8_t>(background);
    command.invert = isInverted ? 1 : 0;
    command.blink = isBlinking ? 1 : 0;
    WriteCommand(0xA3, {reinterpret_cast<uint8_t*>(&command), sizeof(command)});
}

void OsdModule::DisplayCharacter(uint8_t row, uint8_t column, uint8_t characterCode)
{
    // row: 0-11 (NTSC) / 0-15 (PAL); column: 0-29. Range depends on the currently
    // configured video standard, so it isn't asserted here.
    const std::array<uint8_t, 3> argument{row, column, characterCode};
    WriteCommand(0xA4, argument);
}

void OsdModule::DisplayLine(uint8_t row, uint8_t column, std::string_view text)
{
    assert(text.size() <= Config::OSD_MAX_TEXT_SIZE);

    std::array<uint8_t, 2 + Config::OSD_MAX_TEXT_SIZE + 1> argument{};
    argument[0] = row;
    argument[1] = column;
    std::copy(text.begin(), text.end(), argument.begin() + 2);
    argument[2 + text.size()] = 0x00; // terminating byte required by the protocol

    WriteCommand(0xA5, std::span<const uint8_t>(argument.data(), 2 + text.size() + 1));
}

void OsdModule::DisplayUnformattedLine(uint8_t row, uint8_t column, std::string_view text)
{
    assert(text.size() <= Config::OSD_MAX_TEXT_SIZE);

    std::array<uint8_t, 2 + Config::OSD_MAX_TEXT_SIZE + 1> argument{};
    argument[0] = row;
    argument[1] = column;
    std::copy(text.begin(), text.end(), argument.begin() + 2);
    argument[2 + text.size()] = 0x00;

    WriteCommand(0xA6, std::span<const uint8_t>(argument.data(), 2 + text.size() + 1));
}

void OsdModule::ClearScreen()
{
    constexpr uint8_t confirmationByte = 0xA7;
    WriteCommand(0xA7, {&confirmationByte, 1});
}

void OsdModule::SetOffset(int8_t verticalOffsetPx, int8_t horizontalOffsetPx)
{
    assert(verticalOffsetPx >= -15 && verticalOffsetPx <= 16);
    assert(horizontalOffsetPx >= -32 && horizontalOffsetPx <= 31);

    const std::array<uint8_t, 2> argument{
        static_cast<uint8_t>(verticalOffsetPx),
        static_cast<uint8_t>(horizontalOffsetPx)};
    WriteCommand(0xA8, argument);
}

void OsdModule::SetRowBrightness(uint8_t row, BlackLevel blackLevel, WhiteLevel whiteLevel)
{
    SetRowBrightnessRawCommand command{};
    command.row = row;
    command.blackLevel = static_cast<uint8_t>(blackLevel);
    command.whiteLevel = static_cast<uint8_t>(whiteLevel);
    WriteCommand(0xA9, {reinterpret_cast<uint8_t*>(&command), sizeof(command)});
}

void OsdModule::ConfigureGlobalDisplayParams(uint8_t sharpness, BackgroundBrightness backgroundBrightness,
                                              BlinkingPeriod blinkingPeriod, BlinkingDutyCycle blinkingDutyCycle)
{
    assert(sharpness <= 35);

    ConfigureGlobalDisplayParamsRawCommand secondByte{};
    secondByte.backgroundBrightness = static_cast<uint8_t>(backgroundBrightness);
    secondByte.blinkingPeriod = static_cast<uint8_t>(blinkingPeriod);
    secondByte.blinkingDutyCycle = static_cast<uint8_t>(blinkingDutyCycle);

    const std::array<uint8_t, 2> argument{
        std::clamp<uint8_t>(sharpness, 0, 35),
        *reinterpret_cast<uint8_t*>(&secondByte)};
    WriteCommand(0xAA, argument);
}

void OsdModule::OverwriteCharacterImage(uint8_t characterCode, const CharacterBitmap& bitmap)
{
    constexpr uint8_t pixelsPerByte = 4;
    constexpr uint8_t bitsPerPixel = 2;
    constexpr uint8_t bytesPerRow = CharacterBitmapWidthPx / pixelsPerByte;
    constexpr uint8_t imageByteCount = CharacterBitmapHeightPx * bytesPerRow;

    std::array<uint8_t, 1 + imageByteCount> argument{};
    argument[0] = characterCode;

    for (uint8_t row = 0; row < CharacterBitmapHeightPx; ++row)
    {
        for (uint8_t byteInRow = 0; byteInRow < bytesPerRow; ++byteInRow)
        {
            const uint8_t firstColumn = static_cast<uint8_t>(byteInRow * pixelsPerByte);
            uint8_t packedByte = 0;
            for (uint8_t pixelInByte = 0; pixelInByte < pixelsPerByte; ++pixelInByte)
            {
                const uint8_t column = static_cast<uint8_t>(firstColumn + pixelInByte);
                const auto pixelColor = static_cast<uint8_t>(bitmap[row][column]);
                packedByte |= static_cast<uint8_t>(pixelColor << (pixelInByte * bitsPerPixel));
            }
            argument[1 + (row * bytesPerRow) + byteInRow] = packedByte;
        }
    }

    WriteCommand(0xAC, argument);
}

void OsdModule::RestoreDefaultSettings(bool restoreOutputParams, bool restoreUartParams, bool restoreFont, bool restoreInitSequence)
{
    RestoreDefaultSettingsRawCommand command{};
    command.restoreOutputParams = restoreOutputParams ? 1 : 0;
    command.restoreUartParams = restoreUartParams ? 1 : 0;
    command.restoreFont = restoreFont ? 1 : 0;
    command.restoreInitSequence = restoreInitSequence ? 1 : 0;
    WriteCommand(0xAD, {reinterpret_cast<uint8_t*>(&command), sizeof(command)});
}

std::optional<uint8_t> OsdModule::SaveCommandSequence(std::span<const uint8_t> commandSequence, uint32_t timeoutMs)
{
    assert(commandSequence.size() + 1 <= Config::OSD_MODULE_COMMAND_SEQUENCE_SIZE);

    std::array<uint8_t, Config::OSD_MODULE_COMMAND_SEQUENCE_SIZE> argument{};
    std::copy(commandSequence.begin(), commandSequence.end(), argument.begin());
    argument[commandSequence.size()] = 0x00;

    WriteCommand(0xAE, std::span<const uint8_t>(argument.data(), commandSequence.size() + 1));

    uint8_t response = 0;
    if (!m_reader.ReadBytes({&response, 1}, timeoutMs))
        return std::nullopt;
    return response;
}

void OsdModule::ExecuteCommandSequence(uint8_t sequenceNumber)
{
    WriteCommand(0xAF, {&sequenceNumber, 1});
}

void OsdModule::DeleteCommandSequence(uint8_t sequenceNumber)
{
    WriteCommand(0xB0, {&sequenceNumber, 1});
}

void OsdModule::ClearCommandBuffer()
{
    constexpr uint8_t confirmationByte = 0xB1;
    WriteCommand(0xB1, {&confirmationByte, 1});
}

void OsdModule::Delay(uint8_t delayMs)
{
    WriteCommand(0xB2, {&delayMs, 1});
}

std::optional<InputSignalStatus> OsdModule::GetInputSignalStatus(uint32_t timeoutMs)
{
    WriteCommand(0xAB, {});

    uint8_t status = 0;
    if (!m_reader.ReadBytes({&status, 1}, timeoutMs))
        return std::nullopt;

    switch (static_cast<InputSignalStatus>(status))
    {
        case InputSignalStatus::NoSignal:
        case InputSignalStatus::Pal:
        case InputSignalStatus::Ntsc:
            return static_cast<InputSignalStatus>(status);
        default:
            return std::nullopt;
    }
}
