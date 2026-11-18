#define MA_NO_THREADING
#define MA_NO_DEVICE_IO
#include "../common/common.c"

/*
Tests for the AIFF/AIFC encoder. The encoder writes through the generic ma_encoder callback API
into an in-memory stream, after which a ma_decoder is initialized over the produced bytes to verify
the round trip is sample-exact and the headers are well formed.
*/

typedef struct
{
    ma_uint8* pData;
    size_t capacity;
    size_t cursor;
    size_t size;       /* High-water mark; emulates the size of an underlying file. */
    ma_bool32 seekable;
} test_memory_stream;

static ma_result test_memory_stream_write(ma_encoder* pEncoder, const void* pBufferIn, size_t bytesToWrite, size_t* pBytesWritten)
{
    test_memory_stream* pStream;

    (void)pEncoder;
    pStream = (test_memory_stream*)pEncoder->pUserData;
    MA_ASSERT(pStream != NULL);

    if (pStream->cursor + bytesToWrite > pStream->capacity) {
        if (pBytesWritten != NULL) {
            *pBytesWritten = 0;
        }
        return MA_NO_SPACE;
    }

    MA_COPY_MEMORY(pStream->pData + pStream->cursor, pBufferIn, bytesToWrite);
    pStream->cursor += bytesToWrite;
    if (pStream->cursor > pStream->size) {
        pStream->size = pStream->cursor;
    }

    if (pBytesWritten != NULL) {
        *pBytesWritten = bytesToWrite;
    }

    return MA_SUCCESS;
}

/* Emulates lseek(): moves the cursor, including SEEK_END relative to the current file high-water mark. */
static ma_result test_memory_stream_seek(ma_encoder* pEncoder, ma_int64 offset, ma_seek_origin origin)
{
    test_memory_stream* pStream;
    ma_int64 absolutePos;

    pStream = (test_memory_stream*)pEncoder->pUserData;
    MA_ASSERT(pStream != NULL);

    if (!pStream->seekable) {
        return MA_NOT_IMPLEMENTED;
    }

    if (origin == ma_seek_origin_start) {
        absolutePos = offset;
    } else if (origin == ma_seek_origin_current) {
        absolutePos = (ma_int64)pStream->cursor + offset;
    } else {
        absolutePos = (ma_int64)pStream->size + offset;
    }

    if (absolutePos < 0 || (size_t)absolutePos > pStream->capacity) {
        return MA_INVALID_ARGS;
    }

    pStream->cursor = (size_t)absolutePos;
    return MA_SUCCESS;
}

static ma_result test_memory_stream_decoder_read(ma_decoder* pDecoder, void* pBufferOut, size_t bytesToRead, size_t* pBytesRead)
{
    test_memory_stream* pStream;

    pStream = (test_memory_stream*)pDecoder->pUserData;
    MA_ASSERT(pStream != NULL);

    if (pStream->cursor >= pStream->size) {
        if (pBytesRead != NULL) {
            *pBytesRead = 0;
        }
        return MA_AT_END;
    }

    if (pStream->cursor + bytesToRead > pStream->size) {
        bytesToRead = pStream->size - pStream->cursor;
    }

    MA_COPY_MEMORY(pBufferOut, pStream->pData + pStream->cursor, bytesToRead);
    pStream->cursor += bytesToRead;

    if (pBytesRead != NULL) {
        *pBytesRead = bytesToRead;
    }

    return MA_SUCCESS;
}

static ma_result test_memory_stream_decoder_seek(ma_decoder* pDecoder, ma_int64 offset, ma_seek_origin origin)
{
    test_memory_stream* pStream;
    ma_int64 absolutePos;

    pStream = (test_memory_stream*)pDecoder->pUserData;
    MA_ASSERT(pStream != NULL);

    if (origin == ma_seek_origin_start) {
        absolutePos = offset;
    } else if (origin == ma_seek_origin_current) {
        absolutePos = (ma_int64)pStream->cursor + offset;
    } else {
        absolutePos = (ma_int64)pStream->size + offset;
    }

    if (absolutePos < 0 || (size_t)absolutePos > pStream->size) {
        return MA_INVALID_ARGS;
    }

    pStream->cursor = (size_t)absolutePos;
    return MA_SUCCESS;
}

static ma_uint32 test_be32(const ma_uint8* pData)
{
    return ((ma_uint32)pData[0] << 24) | ((ma_uint32)pData[1] << 16) | ((ma_uint32)pData[2] << 8) | (ma_uint32)pData[3];
}

static ma_uint16 test_be16(const ma_uint8* pData)
{
    return (ma_uint16)(((ma_uint16)pData[0] << 8) | (ma_uint16)pData[1]);
}

#define TEST_INT64_MIN   ((ma_int64)((ma_uint64)0x80000000 << 32))
#define TEST_INT64_MAX   ((ma_int64)(((ma_uint64)0x7FFFFFFF << 32) | 0xFFFFFFFF))

/* Reads back the 80-bit extended sample rate the way the AIFF decoder does and checks it matches exactly. */
static ma_int64 test_extended_to_s64(const ma_uint8* pData)
{
    ma_uint32 exponent = ((ma_uint32)pData[0] << 8) | pData[1];
    ma_uint64 hi = ((ma_uint64)pData[2] << 24) | ((ma_uint64)pData[3] << 16) | ((ma_uint64)pData[4] << 8) | (ma_uint64)pData[5];
    ma_uint64 lo = ((ma_uint64)pData[6] << 24) | ((ma_uint64)pData[7] << 16) | ((ma_uint64)pData[8] << 8) | (ma_uint64)pData[9];
    ma_uint64 significand = (hi << 32) | lo;
    int sign = (int)(exponent >> 15);

    exponent &= 0x7FFF;
    if (exponent == 0 && significand == 0) {
        return 0;
    }

    exponent -= 16383;
    if (exponent >= 63) {
        return sign ? TEST_INT64_MIN : TEST_INT64_MAX;
    }
    if (exponent < 1) {
        return 0;
    }

    significand >>= (63 - exponent);
    if (sign) {
        return -(ma_int64)significand;
    } else {
        return (ma_int64)significand;
    }
}

/*
Byte offsets of each field in the produced file.

  0  "FORM"          12 "COMM"          AIFF: 38 "SSND"   AIFC: 44 "SSND"
  4  form size       16 COMM size             42 size             48 size
  8  "AIFF"/"AIFC"   20 channels              46 offset           52 offset
                     22 frame count           50 block            56 block
                     26 bits                  54 data             60 data
                     28 sample rate (10B)
*/
#define TEST_OFF_FORM_TYPE            8
#define TEST_OFF_FORM_SIZE            4
#define TEST_OFF_COMM_ID             12
#define TEST_OFF_COMM_SIZE           16
#define TEST_OFF_COMM_CHANNELS       20
#define TEST_OFF_COMM_FRAME_COUNT    22
#define TEST_OFF_COMM_BITS           26
#define TEST_OFF_COMM_RATE           28
#define TEST_OFF_AIFC_CODE           38
#define TEST_OFF_SSND_ID_AIFF        38
#define TEST_OFF_SSND_SIZE_AIFF      42
#define TEST_OFF_SSND_DATA_AIFF      54
#define TEST_OFF_SSND_ID_AIFC        44
#define TEST_OFF_SSND_SIZE_AIFC      48
#define TEST_OFF_SSND_DATA_AIFC      60

/*
Encodes `frameCount` frames from `pFramesIn` and decodes them back. Returns MA_SUCCESS only when the
decoded samples are byte-for-byte identical to the input. Also validates the structural header.
*/
static ma_result test_aiff_roundtrip(ma_format format, ma_uint32 channels, ma_uint32 sampleRate, const void* pFramesIn, ma_uint64 frameCount, ma_bool32 expectPadByte)
{
    ma_result result;
    ma_encoder_config encoderConfig;
    ma_encoder encoder;
    test_memory_stream stream;
    ma_decoder_config decoderConfig;
    ma_decoder decoder;
    ma_uint64 framesRead = 0;
    ma_uint64 lengthInFrames = 0;
    ma_uint32 bytesPerSample;
    ma_uint64 totalSamples;
    size_t dataSize;
    ma_uint8* pDecoded;
    ma_bool32 isAIFC;
    ma_uint32 ssndIdPos;
    ma_uint32 ssndSizePos;
    ma_uint32 ssndDataPos;
    ma_uint32 ssndBodySize;
    ma_uint32 expectedFileSize;

    MA_ASSERT(pFramesIn != NULL);

    isAIFC = (format == ma_format_f32) ? MA_TRUE : MA_FALSE;
    bytesPerSample = ma_get_bytes_per_sample(format);
    totalSamples = frameCount * (ma_uint64)channels;
    dataSize = (size_t)(totalSamples * (ma_uint64)bytesPerSample);

    if (isAIFC) {
        ssndIdPos = TEST_OFF_SSND_ID_AIFC;
        ssndSizePos = TEST_OFF_SSND_SIZE_AIFC;
        ssndDataPos = TEST_OFF_SSND_DATA_AIFC;
    } else {
        ssndIdPos = TEST_OFF_SSND_ID_AIFF;
        ssndSizePos = TEST_OFF_SSND_SIZE_AIFF;
        ssndDataPos = TEST_OFF_SSND_DATA_AIFF;
    }

    stream.pData = (ma_uint8*)ma_malloc(4096, NULL);
    if (stream.pData == NULL) {
        return MA_OUT_OF_MEMORY;
    }
    stream.capacity = 4096;
    stream.cursor = 0;
    stream.size = 0;
    stream.seekable = MA_TRUE;

    encoderConfig = ma_encoder_config_init(ma_encoding_format_aiff, format, channels, sampleRate);
    result = ma_encoder_init(test_memory_stream_write, test_memory_stream_seek, &stream, &encoderConfig, &encoder);
    if (result != MA_SUCCESS) {
        ma_free(stream.pData, NULL);
        return result;
    }

    {
        ma_uint64 framesWritten = 0;
        result = ma_encoder_write_pcm_frames(&encoder, pFramesIn, frameCount, &framesWritten);
        if (result != MA_SUCCESS || framesWritten != frameCount) {
            ma_encoder_uninit(&encoder);
            ma_free(stream.pData, NULL);
            return MA_ERROR;
        }
    }

    ma_encoder_uninit(&encoder);

    /* Structural validation of the generated file. */
    if (stream.size != (size_t)ssndDataPos + dataSize + (expectPadByte ? 1u : 0u)) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (memcmp(stream.pData + 0, "FORM", 4) != 0) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (memcmp(stream.pData + TEST_OFF_FORM_TYPE, isAIFC ? "AIFC" : "AIFF", 4) != 0) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (test_be32(stream.pData + TEST_OFF_FORM_SIZE) != (ma_uint32)(stream.size - 8)) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (memcmp(stream.pData + TEST_OFF_COMM_ID, "COMM", 4) != 0) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (test_be32(stream.pData + TEST_OFF_COMM_SIZE) != (isAIFC ? 24u : 18u)) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (test_be16(stream.pData + TEST_OFF_COMM_CHANNELS) != (ma_uint16)channels) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (test_be32(stream.pData + TEST_OFF_COMM_FRAME_COUNT) != (ma_uint32)frameCount) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (test_be16(stream.pData + TEST_OFF_COMM_BITS) != (ma_uint16)(bytesPerSample * 8)) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if ((ma_uint32)test_extended_to_s64(stream.pData + TEST_OFF_COMM_RATE) != sampleRate) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (isAIFC) {
        if (memcmp(stream.pData + TEST_OFF_AIFC_CODE, "fl32", 4) != 0) {
            ma_free(stream.pData, NULL);
            return MA_INVALID_FILE;
        }
        if (stream.pData[TEST_OFF_AIFC_CODE + 4] != 0 || stream.pData[TEST_OFF_AIFC_CODE + 5] != 0) {
            ma_free(stream.pData, NULL);
            return MA_INVALID_FILE;
        }
    }
    if (memcmp(stream.pData + ssndIdPos, "SSND", 4) != 0) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    ssndBodySize = 8 + (ma_uint32)dataSize;
    if (test_be32(stream.pData + ssndSizePos) != ssndBodySize) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (test_be32(stream.pData + ssndSizePos + 4) != 0 || test_be32(stream.pData + ssndSizePos + 8) != 0) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }
    if (expectPadByte) {
        if (stream.pData[stream.size - 1] != 0) {
            ma_free(stream.pData, NULL);
            return MA_INVALID_FILE;
        }
    }
    expectedFileSize = ssndDataPos + (ma_uint32)dataSize + (expectPadByte ? 1u : 0u);
    if (stream.size != expectedFileSize) {
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }

    /* Decode the bytes back and compare samples. */
    stream.cursor = 0;
    decoderConfig = ma_decoder_config_init(format, channels, sampleRate);
    decoderConfig.encodingFormat = ma_encoding_format_aiff;
    result = ma_decoder_init(test_memory_stream_decoder_read, test_memory_stream_decoder_seek, &stream, &decoderConfig, &decoder);
    if (result != MA_SUCCESS) {
        ma_free(stream.pData, NULL);
        return result;
    }

    result = ma_decoder_get_length_in_pcm_frames(&decoder, &lengthInFrames);
    if (result != MA_SUCCESS || lengthInFrames != frameCount) {
        ma_decoder_uninit(&decoder);
        ma_free(stream.pData, NULL);
        return MA_INVALID_FILE;
    }

    pDecoded = (ma_uint8*)ma_malloc(dataSize, NULL);
    if (pDecoded == NULL) {
        ma_decoder_uninit(&decoder);
        ma_free(stream.pData, NULL);
        return MA_OUT_OF_MEMORY;
    }

    result = ma_decoder_read_pcm_frames(&decoder, pDecoded, frameCount, &framesRead);
    if (result != MA_SUCCESS || framesRead != frameCount) {
        ma_free(pDecoded, NULL);
        ma_decoder_uninit(&decoder);
        ma_free(stream.pData, NULL);
        return MA_ERROR;
    }

    if (memcmp(pDecoded, pFramesIn, dataSize) != 0) {
        ma_free(pDecoded, NULL);
        ma_decoder_uninit(&decoder);
        ma_free(stream.pData, NULL);
        return MA_INVALID_DATA;
    }

    ma_free(pDecoded, NULL);
    ma_decoder_uninit(&decoder);
    ma_free(stream.pData, NULL);
    return MA_SUCCESS;
}

static ma_result test_aiff_roundtrip__all_formats(void)
{
    ma_result result = MA_SUCCESS;
    ma_uint32 iFrame;

    /* Odd total frame count with a mono 8-bit stream makes the SSND payload odd, exercising the pad byte. */
    ma_uint8  inputU8[7];
    ma_int16  inputS16[2 * 11];
    ma_uint8  inputS24[3 * 9];
    ma_int32  inputS32[2 * 13];
    float     inputF32[2 * 17];

    printf("Round trip: u8 (odd payload, pad byte)\n");
    for (iFrame = 0; iFrame < ma_countof(inputU8); iFrame += 1) {
        inputU8[iFrame] = (ma_uint8)(iFrame * 37u);  /* Includes values on both sides of the 128 bias. */
    }
    result = test_aiff_roundtrip(ma_format_u8, 1, 44100, inputU8, ma_countof(inputU8), MA_TRUE);
    if (result != MA_SUCCESS) {
        goto done;
    }

    printf("Round trip: s16 stereo\n");
    for (iFrame = 0; iFrame < ma_countof(inputS16); iFrame += 1) {
        inputS16[iFrame] = (ma_int16)(iFrame * 257 - 3000);
    }
    result = test_aiff_roundtrip(ma_format_s16, 2, 48000, inputS16, ma_countof(inputS16) / 2, MA_FALSE);
    if (result != MA_SUCCESS) {
        goto done;
    }

    printf("Round trip: s24 (3 bytes per sample, odd payload)\n");
    for (iFrame = 0; iFrame < ma_countof(inputS24); iFrame += 1) {
        inputS24[iFrame] = (ma_uint8)(iFrame * 13u + 1);
    }
    result = test_aiff_roundtrip(ma_format_s24, 1, 96000, inputS24, ma_countof(inputS24) / 3, MA_TRUE);
    if (result != MA_SUCCESS) {
        goto done;
    }

    printf("Round trip: s32 stereo\n");
    for (iFrame = 0; iFrame < ma_countof(inputS32); iFrame += 1) {
        inputS32[iFrame] = (ma_int32)(iFrame * 2654435761u);
    }
    result = test_aiff_roundtrip(ma_format_s32, 2, 192000, inputS32, ma_countof(inputS32) / 2, MA_FALSE);
    if (result != MA_SUCCESS) {
        goto done;
    }

    printf("Round trip: f32 stereo (AIFC, fl32)\n");
    for (iFrame = 0; iFrame < ma_countof(inputF32); iFrame += 1) {
        inputF32[iFrame] = ((float)(iFrame - 17) / 31.0f) + (iFrame & 1 ? 0.25f : -0.5f);
    }
    result = test_aiff_roundtrip(ma_format_f32, 2, 22050, inputF32, ma_countof(inputF32) / 2, MA_FALSE);
    if (result != MA_SUCCESS) {
        goto done;
    }

done:
    if (result != MA_SUCCESS) {
        printf("FAILED (%d)\n", (int)result);
    } else {
        printf("PASSED\n");
    }

    return result;
}

/*
Exercises the 80-bit extended precision sample rate field across the range of practical rates,
including the decoder's upper limit. Each rate must survive the encode/decode round trip exactly
and decode back from the raw 10-byte field to the same integer value.
*/
static ma_result test_aiff_sample_rates(void)
{
    static const ma_uint32 rates[] = {8000u, 11025u, 22050u, 44100u, 48000u, 88200u, 96000u, 176400u, 192000u, 384000u};
    ma_uint32 iRate;
    ma_result result = MA_SUCCESS;
    ma_int16 input[4];
    ma_uint32 iSample;

    printf("80-bit extended sample rates\n");

    for (iSample = 0; iSample < ma_countof(input); iSample += 1) {
        input[iSample] = (ma_int16)(iSample * 9001 - 12000);
    }

    for (iRate = 0; iRate < ma_countof(rates); iRate += 1) {
        ma_encoder_config encoderConfig;
        ma_encoder encoder;
        test_memory_stream stream;
        ma_uint64 framesWritten = 0;

        stream.pData = (ma_uint8*)ma_malloc(1024, NULL);
        if (stream.pData == NULL) {
            return MA_OUT_OF_MEMORY;
        }
        stream.capacity = 1024;
        stream.cursor = 0;
        stream.size = 0;
        stream.seekable = MA_TRUE;

        encoderConfig = ma_encoder_config_init(ma_encoding_format_aiff, ma_format_s16, 1, rates[iRate]);
        result = ma_encoder_init(test_memory_stream_write, test_memory_stream_seek, &stream, &encoderConfig, &encoder);
        if (result != MA_SUCCESS) {
            ma_free(stream.pData, NULL);
            goto done;
        }

        result = ma_encoder_write_pcm_frames(&encoder, input, ma_countof(input), &framesWritten);
        ma_encoder_uninit(&encoder);
        if (result != MA_SUCCESS || framesWritten != ma_countof(input)) {
            ma_free(stream.pData, NULL);
            result = MA_ERROR;
            goto done;
        }

        /* The extended field is at a fixed offset and must decode back to the exact requested rate. */
        if ((ma_uint32)test_extended_to_s64(stream.pData + TEST_OFF_COMM_RATE) != rates[iRate]) {
            ma_free(stream.pData, NULL);
            result = MA_INVALID_DATA;
            goto done;
        }

        ma_free(stream.pData, NULL);
    }

done:
    if (result != MA_SUCCESS) {
        printf("FAILED (%d)\n", (int)result);
    } else {
        printf("PASSED\n");
    }

    return result;
}

static ma_result test_aiff_nonseekable(void)
{
    ma_result result;
    ma_encoder_config encoderConfig;
    ma_encoder encoder;
    test_memory_stream stream;
    ma_int16 input[8];
    ma_uint32 iSample;

    printf("Non-seekable output is rejected with a distinct error\n");

    for (iSample = 0; iSample < ma_countof(input); iSample += 1) {
        input[iSample] = (ma_int16)iSample;
    }

    stream.pData = (ma_uint8*)ma_malloc(4096, NULL);
    if (stream.pData == NULL) {
        return MA_OUT_OF_MEMORY;
    }
    stream.capacity = 4096;
    stream.cursor = 0;
    stream.size = 0;
    stream.seekable = MA_FALSE;

    encoderConfig = ma_encoder_config_init(ma_encoding_format_aiff, ma_format_s16, 1, 48000);
    result = ma_encoder_init(test_memory_stream_write, test_memory_stream_seek, &stream, &encoderConfig, &encoder);
    ma_free(stream.pData, NULL);

    if (result != MA_NOT_IMPLEMENTED) {
        printf("FAILED (expected MA_NOT_IMPLEMENTED, got %d)\n", (int)result);
        return MA_ERROR;
    }

    printf("PASSED\n");
    return MA_SUCCESS;
}

static ma_result test_aiff_unsupported_format(void)
{
    ma_result result;
    ma_encoder_config encoderConfig;
    ma_encoder encoder;
    test_memory_stream stream;

    printf("Unsupported sample format is rejected with a distinct error\n");

    stream.pData = (ma_uint8*)ma_malloc(256, NULL);
    if (stream.pData == NULL) {
        return MA_OUT_OF_MEMORY;
    }
    stream.capacity = 256;
    stream.cursor = 0;
    stream.size = 0;
    stream.seekable = MA_TRUE;

    /* ma_format_unknown has no AIFF representation and must be distinguishable from seek failures. */
    encoderConfig = ma_encoder_config_init(ma_encoding_format_aiff, ma_format_unknown, 1, 48000);
    result = ma_encoder_init(test_memory_stream_write, test_memory_stream_seek, &stream, &encoderConfig, &encoder);
    ma_free(stream.pData, NULL);

    if (result != MA_INVALID_ARGS) {
        printf("FAILED (expected MA_INVALID_ARGS, got %d)\n", (int)result);
        return MA_ERROR;
    }

    printf("PASSED\n");
    return MA_SUCCESS;
}

static ma_result test_wav_encoder_regression(void)
{
    ma_result result;
    ma_encoder_config encoderConfig;
    ma_encoder encoder;
    test_memory_stream stream;
    ma_decoder_config decoderConfig;
    ma_decoder decoder;
    ma_int16 input[32];
    ma_int16 output[32];
    ma_uint64 framesWritten = 0;
    ma_uint64 framesRead = 0;
    ma_uint32 iSample;

    printf("WAV path regression\n");

    for (iSample = 0; iSample < ma_countof(input); iSample += 1) {
        input[iSample] = (ma_int16)(iSample * 1000 - 16000);
    }

    stream.pData = (ma_uint8*)ma_malloc(4096, NULL);
    if (stream.pData == NULL) {
        return MA_OUT_OF_MEMORY;
    }
    stream.capacity = 4096;
    stream.cursor = 0;
    stream.size = 0;
    stream.seekable = MA_TRUE;

    encoderConfig = ma_encoder_config_init(ma_encoding_format_wav, ma_format_s16, 2, 44100);
    result = ma_encoder_init(test_memory_stream_write, test_memory_stream_seek, &stream, &encoderConfig, &encoder);
    if (result != MA_SUCCESS) {
        ma_free(stream.pData, NULL);
        printf("FAILED (init %d)\n", (int)result);
        return MA_ERROR;
    }

    result = ma_encoder_write_pcm_frames(&encoder, input, ma_countof(input) / 2, &framesWritten);
    ma_encoder_uninit(&encoder);
    if (result != MA_SUCCESS || framesWritten != ma_countof(input) / 2) {
        ma_free(stream.pData, NULL);
        printf("FAILED (write)\n");
        return MA_ERROR;
    }

    if (memcmp(stream.pData, "RIFF", 4) != 0 || memcmp(stream.pData + 8, "WAVE", 4) != 0) {
        ma_free(stream.pData, NULL);
        printf("FAILED (header)\n");
        return MA_ERROR;
    }

    stream.cursor = 0;
    decoderConfig = ma_decoder_config_init(ma_format_s16, 2, 44100);
    result = ma_decoder_init(test_memory_stream_decoder_read, test_memory_stream_decoder_seek, &stream, &decoderConfig, &decoder);
    if (result != MA_SUCCESS) {
        ma_free(stream.pData, NULL);
        printf("FAILED (decoder init %d)\n", (int)result);
        return MA_ERROR;
    }

    result = ma_decoder_read_pcm_frames(&decoder, output, ma_countof(output) / 2, &framesRead);
    ma_decoder_uninit(&decoder);
    ma_free(stream.pData, NULL);
    if (result != MA_SUCCESS || framesRead != ma_countof(output) / 2 || memcmp(input, output, sizeof(input)) != 0) {
        printf("FAILED (round trip)\n");
        return MA_ERROR;
    }

    printf("PASSED\n");
    return MA_SUCCESS;
}

int test_entry__aiff_encode(int argc, char** argv)
{
    ma_result result;
    ma_bool32 hasError = MA_FALSE;

    (void)argc;
    (void)argv;

    result = test_aiff_roundtrip__all_formats();
    if (result != MA_SUCCESS) {
        hasError = MA_TRUE;
    }

    result = test_aiff_sample_rates();
    if (result != MA_SUCCESS) {
        hasError = MA_TRUE;
    }

    result = test_aiff_nonseekable();
    if (result != MA_SUCCESS) {
        hasError = MA_TRUE;
    }

    result = test_aiff_unsupported_format();
    if (result != MA_SUCCESS) {
        hasError = MA_TRUE;
    }

    result = test_wav_encoder_regression();
    if (result != MA_SUCCESS) {
        hasError = MA_TRUE;
    }

    if (hasError) {
        return -1;
    } else {
        return 0;
    }
}

int main(int argc, char** argv)
{
    ma_register_test("AIFF/AIFC Encoding", test_entry__aiff_encode);

    return ma_run_tests(argc, argv);
}
