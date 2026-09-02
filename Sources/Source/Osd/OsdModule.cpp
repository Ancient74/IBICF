#include "OsdModule.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <vector>

namespace
{
    constexpr uint32_t QueueDepthBytes = 64;
}

OsdResponseReader::OsdResponseReader()
{
    const osMessageQueueAttr_t queueAttributes = {
        .name = "OsdResponseReader",
        .cb_mem = &m_queueBuffer,
        .cb_size = sizeof(m_queueBuffer),
        .mq_mem = m_queueStorage,
        .mq_size = sizeof(m_queueStorage),
    };
    m_queue = osMessageQueueNew(QueueDepthBytes, sizeof(uint8_t), &queueAttributes);
}

OsdResponseReader::~OsdResponseReader()
{
    osMessageQueueDelete(m_queue);
}

void OsdResponseReader::OnByte(uint8_t byte)
{
    osMessageQueuePut(m_queue, &byte, 0, 0);
}

std::optional<std::vector<uint8_t>> OsdResponseReader::ReadBytes(size_t byteCount, uint32_t timeoutMs)
{
    std::vector<uint8_t> result;
    result.reserve(byteCount);
    for (size_t i = 0; i < byteCount; i++)
    {
        uint8_t byte = 0;
        if (osMessageQueueGet(m_queue, &byte, nullptr, timeoutMs) != osOK)
            return std::nullopt;

        result.push_back(byte);
    }
    return result;
}

OsdModule::OsdModule(std::unique_ptr<IWriter> writer, std::shared_ptr<IOsdResponseReader> reader)
    : m_writer(std::move(writer))
    , m_reader(std::move(reader))
{
}

void OsdModule::WriteCommand(uint8_t commandId, std::span<const uint8_t> argument)
{
    m_writer->Write({&commandId, 1});
    if (!argument.empty())
    {
        m_writer->Write(argument);
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
    // `text` must already be Cp1251 (Windows-1251) encoded — this is a raw byte
    // copy, not a transcode from UTF-8.
    std::vector<uint8_t> argument;
    argument.reserve(2 + text.size() + 1);
    argument.push_back(row);
    argument.push_back(column);
    argument.insert(argument.end(), text.begin(), text.end());
    argument.push_back(0x00); // terminating byte required by the protocol
    WriteCommand(0xA5, argument);
}

void OsdModule::DisplayUnformattedLine(uint8_t row, uint8_t column, std::string_view text)
{
    // Same encoding caveat as DisplayLine. \n and \t are NOT special here —
    // they're written out as ordinary character codes.
    std::vector<uint8_t> argument;
    argument.reserve(2 + text.size() + 1);
    argument.push_back(row);
    argument.push_back(column);
    argument.insert(argument.end(), text.begin(), text.end());
    argument.push_back(0x00);
    WriteCommand(0xA6, argument);
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
    std::vector<uint8_t> argument(commandSequence.begin(), commandSequence.end());
    argument.push_back(0x00);

    WriteCommand(0xAE, argument);

    auto response = m_reader->ReadBytes(1, timeoutMs);
    if (!response)
    {
        return std::nullopt;
    }
    return response->at(0);
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

    auto response = m_reader->ReadBytes(1, timeoutMs);
    if (!response)
    {
        return std::nullopt;
    }

    const uint8_t status = response->at(0);
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
