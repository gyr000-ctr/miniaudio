/*
A self-contained test for the built-in CAF (Apple Core Audio Format) decoder.

The test synthesizes CAF containers and matching WAV containers entirely in memory (no external
tools or files), decodes both through the public ma_decoder API, and requires sample-for-sample
identical output. It also exercises seeking, the truncation-versus-unknown-codec error split,
container detection, and constant peak working memory.

Build (single translation unit, about 2 seconds):
    gcc tests/caf/test_caf.c -o test_caf -ldl -lm -lpthread -Wall -Wextra -Wpedantic -std=c89
*/
#define MA_NO_THREADING
#define MA_NO_DEVICE_IO
#include "../common/common.c"

/* ------------------------------------------------------------------ */
/* Big-endian writers for CAF assembly.                               */
/* ------------------------------------------------------------------ */

typedef struct
{
    ma_uint8* pData;
    size_t size;
    size_t cap;
} test_buf;

static void test_buf_init(test_buf* pBuf)
{
    pBuf->cap = 1u << 16;
    pBuf->size = 0;
    pBuf->pData = (ma_uint8*)ma_malloc(pBuf->cap, NULL);
    MA_ASSERT(pBuf->pData != NULL);
}

static void test_buf_uninit(test_buf* pBuf)
{
    ma_free(pBuf->pData, NULL);
    pBuf->pData = NULL;
    pBuf->size = 0;
    pBuf->cap = 0;
}

static void test_buf_reserve(test_buf* pBuf, size_t extra)
{
    if (pBuf->size + extra > pBuf->cap) {
        while (pBuf->size + extra > pBuf->cap) {
            pBuf->cap *= 2;
        }
        pBuf->pData = (ma_uint8*)ma_realloc(pBuf->pData, pBuf->cap, NULL);
        MA_ASSERT(pBuf->pData != NULL);
    }
}

static void test_buf_put(test_buf* pBuf, const void* pSrc, size_t n)
{
    test_buf_reserve(pBuf, n);
    MA_COPY_MEMORY(pBuf->pData + pBuf->size, pSrc, n);
    pBuf->size += n;
}

static void test_put_u8(test_buf* pBuf, ma_uint8 v)
{
    test_buf_put(pBuf, &v, sizeof(v));
}

static void test_put_be16(test_buf* pBuf, ma_uint16 v)
{
    ma_uint8 b[2];
    b[0] = (ma_uint8)(v >> 8);
    b[1] = (ma_uint8)(v & 0xFF);
    test_buf_put(pBuf, b, sizeof(b));
}

static void test_put_be32(test_buf* pBuf, ma_uint32 v)
{
    ma_uint8 b[4];
    b[0] = (ma_uint8)((v >> 24) & 0xFF);
    b[1] = (ma_uint8)((v >> 16) & 0xFF);
    b[2] = (ma_uint8)((v >> 8) & 0xFF);
    b[3] = (ma_uint8)(v & 0xFF);
    test_buf_put(pBuf, b, sizeof(b));
}

static void test_put_be64(test_buf* pBuf, ma_uint64 v)
{
    test_put_be32(pBuf, (ma_uint32)(v >> 32));
    test_put_be32(pBuf, (ma_uint32)(v & 0xFFFFFFFFu));
}

static void test_put_fourcc(test_buf* pBuf, const char* p4)
{
    test_put_u8(pBuf, (ma_uint8)p4[0]);
    test_put_u8(pBuf, (ma_uint8)p4[1]);
    test_put_u8(pBuf, (ma_uint8)p4[2]);
    test_put_u8(pBuf, (ma_uint8)p4[3]);
}

static void test_put_be_f64(test_buf* pBuf, double v)
{
    ma_uint64 bits;
    MA_COPY_MEMORY(&bits, &v, sizeof(bits));
    test_put_be64(pBuf, bits);
}

/* Little-endian writers for WAV assembly. */
static void test_put_le16(test_buf* pBuf, ma_uint16 v)
{
    ma_uint8 b[2];
    b[0] = (ma_uint8)(v & 0xFF);
    b[1] = (ma_uint8)((v >> 8) & 0xFF);
    test_buf_put(pBuf, b, sizeof(b));
}

static void test_put_le32(test_buf* pBuf, ma_uint32 v)
{
    ma_uint8 b[4];
    b[0] = (ma_uint8)(v & 0xFF);
    b[1] = (ma_uint8)((v >> 8) & 0xFF);
    b[2] = (ma_uint8)((v >> 16) & 0xFF);
    b[3] = (ma_uint8)((v >> 24) & 0xFF);
    test_buf_put(pBuf, b, sizeof(b));
}

/* ------------------------------------------------------------------ */
/* CAF layout description.                                            */
/* ------------------------------------------------------------------ */

#define TEST_FLAG_FLOAT          0x00000001u
#define TEST_FLAG_BIG_ENDIAN     0x00000002u
#define TEST_FLAG_SIGNED_INT     0x00000004u
#define TEST_FLAG_PACKED         0x00000008u
#define TEST_FLAG_ALIGNED_HIGH   0x00000040u

typedef struct
{
    const char* pName;
    ma_format sampleFormat;     /* Logical source sample format. */
    ma_uint32 bitsPerChannel;
    ma_uint32 bytesPerChannel;
    ma_uint32 flags;
    ma_bool32 includePakt;
} test_caf_layout;

/* Builds a CAF file. pSamples are interleaved native-endian source frames. */
static void test_build_caf(test_buf* pOut, const test_caf_layout* pLayout, ma_uint32 channels, ma_uint32 sampleRate, const void* pSamples, size_t sampleByteSize, ma_uint64 frameCount, ma_bool32 truncate)
{
    ma_uint32 bytesPerPacket = pLayout->bytesPerChannel * channels;
    ma_uint64 dataPayloadSize = (ma_uint64)sampleByteSize;
    ma_uint64 paktFrameCount = frameCount;
    size_t i;
    const ma_uint8* pBytes = (const ma_uint8*)pSamples;

    test_buf_init(pOut);

    /* File header. */
    test_put_fourcc(pOut, "caff");
    test_put_be16(pOut, 1);      /* version */
    test_put_be16(pOut, 0);      /* flags */

    /* desc chunk (32-byte body). */
    test_put_fourcc(pOut, "desc");
    test_put_be64(pOut, 32);
    test_put_fourcc(pOut, "lpcm");
    test_put_be32(pOut, pLayout->flags);
    test_put_be32(pOut, bytesPerPacket);          /* bytesPerPacket */
    test_put_be32(pOut, 1);                        /* framesPerPacket */
    test_put_be32(pOut, channels);                 /* channelsPerFrame */
    test_put_be32(pOut, pLayout->bitsPerChannel);  /* bitsPerChannel */
    test_put_be_f64(pOut, (double)sampleRate);

    /* pakt chunk (for constant-bit-rate LPCM the body is just three int64 values, 24 bytes). */
    if (pLayout->includePakt) {
        test_put_fourcc(pOut, "pakt");
        test_put_be64(pOut, 24);
        test_put_be64(pOut, paktFrameCount);       /* valid frames */
        test_put_be64(pOut, 0);                    /* priming frames */
        test_put_be64(pOut, 0);                    /* remainder frames */
    }

    /* data chunk: edit count u32 then raw sample bytes. */
    test_put_fourcc(pOut, "data");
    test_put_be64(pOut, dataPayloadSize + 4);
    test_put_be32(pOut, 0);                        /* edit count */

    /* Emit samples with the required on-disk byte order. */
    for (i = 0; i < sampleByteSize; i += pLayout->bytesPerChannel) {
        ma_uint32 k;
        if (truncate && i >= sampleByteSize / 2) {
            break;  /* Chop the second half of the payload. */
        }
        for (k = 0; k < pLayout->bytesPerChannel; k += 1) {
            ma_uint32 srcIndex;
            if ((pLayout->flags & TEST_FLAG_BIG_ENDIAN) != 0) {
                srcIndex = i + (pLayout->bytesPerChannel - 1 - k);
            } else {
                srcIndex = i + k;
            }
            test_put_u8(pOut, pBytes[srcIndex]);
        }
    }
}

/* Builds a canonical PCM WAV file for the same native-endian samples. */
static void test_build_wav(test_buf* pOut, ma_format sampleFormat, ma_uint32 channels, ma_uint32 sampleRate, const void* pSamples, size_t sampleByteSize, ma_uint64 frameCount)
{
    ma_uint16 audioFormat = (sampleFormat == ma_format_f32) ? 3u : 1u;
    ma_uint16 bitsPerSample = (ma_uint16)(ma_get_bytes_per_sample(sampleFormat) * 8);
    ma_uint16 blockAlign = (ma_uint16)ma_get_bytes_per_frame(sampleFormat, channels);
    ma_uint32 byteRate = sampleRate * blockAlign;
    ma_uint32 dataSize = (ma_uint32)sampleByteSize;

    test_buf_init(pOut);

    test_put_u8(pOut, 'R'); test_put_u8(pOut, 'I'); test_put_u8(pOut, 'F'); test_put_u8(pOut, 'F');
    test_put_le32(pOut, 36 + dataSize);
    test_put_u8(pOut, 'W'); test_put_u8(pOut, 'A'); test_put_u8(pOut, 'V'); test_put_u8(pOut, 'E');
    test_put_u8(pOut, 'f'); test_put_u8(pOut, 'm'); test_put_u8(pOut, 't'); test_put_u8(pOut, ' ');
    test_put_le32(pOut, 16);
    test_put_le16(pOut, audioFormat);
    test_put_le16(pOut, (ma_uint16)channels);
    test_put_le32(pOut, sampleRate);
    test_put_le32(pOut, byteRate);
    test_put_le16(pOut, blockAlign);
    test_put_le16(pOut, bitsPerSample);
    test_put_u8(pOut, 'd'); test_put_u8(pOut, 'a'); test_put_u8(pOut, 't'); test_put_u8(pOut, 'a');
    test_put_le32(pOut, dataSize);
    test_buf_put(pOut, pSamples, sampleByteSize);

    (void)frameCount;
}

/* ------------------------------------------------------------------ */
/* Sample generation. Deterministic, exercises sign/extremes.         */
/* ------------------------------------------------------------------ */

static ma_int16 g_s16Pattern[] = {
    0, 1, -1, 32767, -32768, 12345, -12345, 256, -256, 100, -100, 32000, -32000, 7, -7, 42, -42, 1000, -1000, 555,
    0, 1, -1, 32767, -32768, 12345, -12345, 256, -256, 100, -100, 32000, -32000, 7, -7, 42, -42, 1000, -1000, 555
};

static ma_uint32 g_testFrameCount = 20;   /* 40 samples arranged as 2-channel frames. */

/* ------------------------------------------------------------------ */
/* Decode helper. Decodes a fully-formed in-memory container.         */
/* ------------------------------------------------------------------ */

static ma_result test_decode_all(const ma_uint8* pData, size_t dataSize, ma_format outFormat, void* pOut, ma_uint64 outCapFrames, ma_uint32* pOutChannels, ma_uint32* pOutRate, ma_uint64* pOutFrames, ma_decoder* pDecoderOut)
{
    ma_decoder_config config;
    ma_decoder decoder;
    ma_result result;
    ma_uint64 framesRead;
    ma_uint64 totalRead = 0;
    ma_uint8* pRunning = (ma_uint8*)pOut;
    ma_uint32 bpf;
    ma_uint32 actualChannels = 0;

    config = ma_decoder_config_init(outFormat, 0, 0);
    result = ma_decoder_init_memory(pData, dataSize, &config, &decoder);
    if (result != MA_SUCCESS) {
        return result;
    }

    result = ma_decoder_get_data_format(&decoder, NULL, &actualChannels, NULL, NULL, 0);
    if (result != MA_SUCCESS) {
        ma_decoder_uninit(&decoder);
        return result;
    }

    if (pOutChannels != NULL) {
        *pOutChannels = actualChannels;
    }
    if (pOutRate != NULL) {
        ma_decoder_get_data_format(&decoder, NULL, NULL, pOutRate, NULL, 0);
    }

    bpf = ma_get_bytes_per_frame(outFormat, actualChannels);

    for (;;) {
        ma_uint64 want = outCapFrames - totalRead;
        if (want == 0) {
            break;
        }
        result = ma_decoder_read_pcm_frames(&decoder, pRunning, want, &framesRead);
        if (framesRead > 0) {
            totalRead += framesRead;
            pRunning += framesRead * bpf;
        }
        /* A short final read returns MA_AT_END together with the remaining frames. */
        if (result == MA_AT_END) {
            result = MA_SUCCESS;
            break;
        }
        if (result != MA_SUCCESS) {
            ma_decoder_uninit(&decoder);
            return result;
        }
        if (framesRead == 0) {
            break;
        }
    }

    if (pOutFrames != NULL) {
        *pOutFrames = totalRead;
    }
    if (pDecoderOut != NULL) {
        *pDecoderOut = decoder;
        return MA_SUCCESS;  /* Caller owns the decoder now. */
    }

    ma_decoder_uninit(&decoder);
    return MA_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Individual checks.                                                 */
/* ------------------------------------------------------------------ */

static ma_result test_caf__s16_endian(void)
{
    /* 16-bit signed integer, both little and big-endian CAF versus a LE WAV reference. */
    test_caf_layout leLayout = { "s16 LE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH, MA_TRUE };
    test_caf_layout beLayout = { "s16 BE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
    test_buf cafLe, cafBe, wav;
    ma_int16 cafOutLe[256], cafOutBe[256], wavOut[256];
    ma_uint64 cafFrames = 0, wavFrames = 0;
    ma_uint32 channels = 2;
    ma_result result;
    size_t sampleBytes = sizeof(g_s16Pattern);

    test_build_caf(&cafLe, &leLayout, channels, 44100, g_s16Pattern, sampleBytes, g_testFrameCount, MA_FALSE);
    test_build_caf(&cafBe, &beLayout, channels, 44100, g_s16Pattern, sampleBytes, g_testFrameCount, MA_FALSE);
    test_build_wav(&wav, ma_format_s16, channels, 44100, g_s16Pattern, sampleBytes, g_testFrameCount);

    result = test_decode_all(cafLe.pData, cafLe.size, ma_format_s16, cafOutLe, 256, NULL, NULL, &cafFrames, NULL);
    MA_ASSERT(result == MA_SUCCESS);
    result = test_decode_all(cafBe.pData, cafBe.size, ma_format_s16, cafOutBe, 256, NULL, NULL, &cafFrames, NULL);
    MA_ASSERT(result == MA_SUCCESS);
    result = test_decode_all(wav.pData, wav.size, ma_format_s16, wavOut, 256, NULL, NULL, &wavFrames, NULL);
    MA_ASSERT(result == MA_SUCCESS);

    MA_ASSERT(cafFrames == g_testFrameCount);
    MA_ASSERT(wavFrames == g_testFrameCount);
    MA_ASSERT((memcmp(cafOutLe, wavOut, sizeof(g_s16Pattern)) == 0));
    MA_ASSERT((memcmp(cafOutBe, wavOut, sizeof(g_s16Pattern)) == 0));

    test_buf_uninit(&cafLe);
    test_buf_uninit(&cafBe);
    test_buf_uninit(&wav);
    return MA_SUCCESS;
}

static ma_result test_caf__s24_packed(void)
{
    /* Packed 24-bit, big-endian on disk, converted through the s24 path and compared to a WAV carrying the same s24 values. */
    test_caf_layout layout = { "s24 BE packed", ma_format_s24, 24, 3, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
    static const ma_int32 values[] = {
        0, 1, -1, 8388607, -8388608, 123456, -123456, 256, -256, 7777777, -7777777, 33,
        0, 1, -1, 8388607, -8388608, 123456, -123456, 256, -256, 7777777, -7777777, 33
    };
    ma_uint8 srcS24[ma_countof(values) * 3];
    ma_uint8 cafOut[256 * 3 * 2];
    ma_uint8 wavOut[256 * 3 * 2];
    test_buf caf, wav;
    ma_uint64 cafFrames = 0, wavFrames = 0;
    ma_uint32 channels = 2;
    ma_uint32 i;
    ma_result result;
    ma_uint32 sampleCount = ma_countof(values);

    /* Pack native s32 values into tightly packed native-endian s24 bytes. */
    for (i = 0; i < sampleCount; i += 1) {
        ma_pcm_sample_s32_to_s24_no_scale(values[i], &srcS24[i * 3]);
    }

    test_build_caf(&caf, &layout, channels, 48000, srcS24, (size_t)sampleCount * 3, sampleCount / channels, MA_FALSE);
    test_build_wav(&wav, ma_format_s24, channels, 48000, srcS24, (size_t)sampleCount * 3, sampleCount / channels);

    result = test_decode_all(caf.pData, caf.size, ma_format_s24, cafOut, 256, NULL, NULL, &cafFrames, NULL);
    MA_ASSERT(result == MA_SUCCESS);
    result = test_decode_all(wav.pData, wav.size, ma_format_s24, wavOut, 256, NULL, NULL, &wavFrames, NULL);
    MA_ASSERT(result == MA_SUCCESS);

    MA_ASSERT(cafFrames == ma_countof(values) / channels);
    MA_ASSERT(cafFrames == wavFrames);
    MA_ASSERT((memcmp(cafOut, wavOut, (size_t)cafFrames * channels * 3) == 0));

    test_buf_uninit(&caf);
    test_buf_uninit(&wav);
    return MA_SUCCESS;
}

static ma_result test_caf__formats_via_f32(void)
{
    /* u8, s32 and f32 CAF variants all compared against equivalent WAV data after f32 conversion. */
    ma_uint8 u8Samples[24];
    ma_int32 s32Samples[24];
    float f32Samples[24];
    ma_uint32 i;
    ma_result result = MA_SUCCESS;

    for (i = 0; i < ma_countof(u8Samples); i += 1) {
        u8Samples[i] = (ma_uint8)(i * 11 + 3);
    }
    for (i = 0; i < ma_countof(s32Samples); i += 1) {
        s32Samples[i] = (ma_int32)i * 90000 - 1000000;
    }
    for (i = 0; i < ma_countof(f32Samples); i += 1) {
        f32Samples[i] = ((float)(int)i / 24.0f) - 0.5f;
    }

    {
        /*
        Unsigned 8-bit, little-endian. Compared at the native u8 level rather than after f32
        conversion because the library intentionally uses different u8->f32 scaling in its generic
        PCM conversion (1/255) and its bundled dr_wav path (1/256); that is a pre-existing library
        convention, not a CAF concern. At the native level both decoders must reproduce the bytes.
        */
        test_caf_layout layout = { "u8 LE", ma_format_u8, 8, 1, TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH, MA_TRUE };
        test_buf caf, wav;
        ma_uint8 cafOut[256], wavOut[256];
        ma_uint64 cf = 0, wf = 0;
        test_build_caf(&caf, &layout, 2, 22050, u8Samples, sizeof(u8Samples), ma_countof(u8Samples) / 2, MA_FALSE);
        test_build_wav(&wav, ma_format_u8, 2, 22050, u8Samples, sizeof(u8Samples), ma_countof(u8Samples) / 2);
        result = test_decode_all(caf.pData, caf.size, ma_format_u8, cafOut, 256, NULL, NULL, &cf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        result = test_decode_all(wav.pData, wav.size, ma_format_u8, wavOut, 256, NULL, NULL, &wf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        MA_ASSERT(cf == wf && cf == ma_countof(u8Samples) / 2);
        MA_ASSERT((memcmp(cafOut, wavOut, (size_t)cf * 2 * sizeof(ma_uint8)) == 0));
        MA_ASSERT((memcmp(cafOut, u8Samples, sizeof(u8Samples)) == 0));
        test_buf_uninit(&caf);
        test_buf_uninit(&wav);
    }

    {
        /* Signed 32-bit, big-endian. */
        test_caf_layout layout = { "s32 BE", ma_format_s32, 32, 4, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
        test_buf caf, wav;
        float cafOut[256], wavOut[256];
        ma_uint64 cf = 0, wf = 0;
        test_build_caf(&caf, &layout, 2, 32000, s32Samples, sizeof(s32Samples), ma_countof(s32Samples) / 2, MA_FALSE);
        test_build_wav(&wav, ma_format_s32, 2, 32000, s32Samples, sizeof(s32Samples), ma_countof(s32Samples) / 2);
        result = test_decode_all(caf.pData, caf.size, ma_format_f32, cafOut, 256, NULL, NULL, &cf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        result = test_decode_all(wav.pData, wav.size, ma_format_f32, wavOut, 256, NULL, NULL, &wf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        MA_ASSERT(cf == wf && cf == ma_countof(s32Samples) / 2);
        MA_ASSERT((memcmp(cafOut, wavOut, (size_t)cf * 2 * sizeof(float)) == 0));
        test_buf_uninit(&caf);
        test_buf_uninit(&wav);
    }

    {
        /* 32-bit float, big-endian on disk. */
        test_caf_layout layout = { "f32 BE", ma_format_f32, 32, 4, TEST_FLAG_FLOAT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
        test_buf caf, wav;
        float cafOut[256], wavOut[256];
        ma_uint64 cf = 0, wf = 0;
        test_build_caf(&caf, &layout, 1, 96000, f32Samples, sizeof(f32Samples), ma_countof(f32Samples), MA_FALSE);
        test_build_wav(&wav, ma_format_f32, 1, 96000, f32Samples, sizeof(f32Samples), ma_countof(f32Samples));
        result = test_decode_all(caf.pData, caf.size, ma_format_f32, cafOut, 256, NULL, NULL, &cf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        result = test_decode_all(wav.pData, wav.size, ma_format_f32, wavOut, 256, NULL, NULL, &wf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        MA_ASSERT(cf == wf && cf == ma_countof(f32Samples));
        MA_ASSERT((memcmp(cafOut, wavOut, (size_t)cf * sizeof(float)) == 0));
        test_buf_uninit(&caf);
        test_buf_uninit(&wav);
    }

    {
        /* 64-bit float, big-endian on disk, widened to f32 on output. Compared against an explicit f32 expectation. */
        double f64Samples[16];
        float expected[16];
        test_caf_layout layout = { "f64 BE", ma_format_f32, 64, 8, TEST_FLAG_FLOAT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
        test_buf caf;
        float cafOut[256];
        ma_uint64 cf = 0;
        ma_uint32 k;
        for (k = 0; k < ma_countof(f64Samples); k += 1) {
            f64Samples[k] = ((double)k / 16.0) - 0.5;
            expected[k] = (float)f64Samples[k];
        }
        test_build_caf(&caf, &layout, 1, 48000, f64Samples, sizeof(f64Samples), ma_countof(f64Samples), MA_FALSE);
        result = test_decode_all(caf.pData, caf.size, ma_format_f32, cafOut, 256, NULL, NULL, &cf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        MA_ASSERT(cf == ma_countof(f64Samples));
        MA_ASSERT((memcmp(cafOut, expected, sizeof(expected)) == 0));
        test_buf_uninit(&caf);
    }

    {
        /* Packed 24-bit little-endian on disk, decoded to s16 and checked against a little-endian s24 WAV. */
        static const ma_int32 values24[] = { 0, 1, -1, 8388607, -8388608, 123456, -123456, 256, -256, 33, -33, 99 };
        ma_uint8 srcS24[ma_countof(values24) * 3];
        test_caf_layout layout = { "s24 LE packed", ma_format_s24, 24, 3, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED, MA_TRUE };
        test_buf caf, wav;
        ma_int16 cafOut[256], wavOut[256];
        ma_uint64 cf = 0, wf = 0;
        ma_uint32 k;
        for (k = 0; k < ma_countof(values24); k += 1) {
            ma_pcm_sample_s32_to_s24_no_scale(values24[k], &srcS24[k * 3]);
        }
        test_build_caf(&caf, &layout, 2, 48000, srcS24, sizeof(srcS24), ma_countof(values24) / 2, MA_FALSE);
        test_build_wav(&wav, ma_format_s24, 2, 48000, srcS24, sizeof(srcS24), ma_countof(values24) / 2);
        result = test_decode_all(caf.pData, caf.size, ma_format_s16, cafOut, 256, NULL, NULL, &cf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        result = test_decode_all(wav.pData, wav.size, ma_format_s16, wavOut, 256, NULL, NULL, &wf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        MA_ASSERT(cf == wf && cf == ma_countof(values24) / 2);
        MA_ASSERT((memcmp(cafOut, wavOut, (size_t)cf * 2 * sizeof(ma_int16)) == 0));
        test_buf_uninit(&caf);
        test_buf_uninit(&wav);
    }

    {
        /*
        Aligned-high 24-bit: each sample occupies a 4-byte native-endian container with the 24 valid
        bits left aligned. The builder emits it big-endian on disk per the layout flags, and the
        decoder must yield the same s16 output as a packed s24 WAV carrying the identical values.
        */
        static const ma_int32 vals[] = { 0, 1, -1, 8388607, -8388608, 123456, -123456, 256 };
        ma_int32 containers[ma_countof(vals)];
        ma_uint8 packedS24[ma_countof(vals) * 3];
        test_caf_layout layout = { "s24 BE aligned-high", ma_format_s24, 24, 4, TEST_FLAG_SIGNED_INT | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
        test_buf caf, wav;
        ma_int16 cafOut[256], wavOut[256];
        ma_uint64 cf = 0, wf = 0;
        ma_uint32 k;
        for (k = 0; k < ma_countof(vals); k += 1) {
            containers[k] = vals[k] << 8;   /* Native-endian container; top 24 bits hold the value. */
            ma_pcm_sample_s32_to_s24_no_scale(vals[k], &packedS24[k * 3]);
        }
        test_build_caf(&caf, &layout, 2, 48000, containers, sizeof(containers), ma_countof(vals) / 2, MA_FALSE);
        test_build_wav(&wav, ma_format_s24, 2, 48000, packedS24, sizeof(packedS24), ma_countof(vals) / 2);
        result = test_decode_all(caf.pData, caf.size, ma_format_s16, cafOut, 256, NULL, NULL, &cf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        result = test_decode_all(wav.pData, wav.size, ma_format_s16, wavOut, 256, NULL, NULL, &wf, NULL);
        MA_ASSERT(result == MA_SUCCESS);
        MA_ASSERT(cf == wf && cf == ma_countof(vals) / 2);
        MA_ASSERT((memcmp(cafOut, wavOut, (size_t)cf * 2 * sizeof(ma_int16)) == 0));
        test_buf_uninit(&caf);
        test_buf_uninit(&wav);
    }

    return result;
}

static ma_result test_caf__length_and_cursor(void)
{
    test_caf_layout layout = { "s16 BE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
    test_buf caf;
    ma_decoder decoder;
    ma_decoder_config config;
    ma_result result;
    ma_uint64 length = 0, cursor = 0;
    ma_int16 tmp[64];
    ma_uint64 framesRead;

    test_build_caf(&caf, &layout, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount, MA_FALSE);

    config = ma_decoder_config_init(ma_format_s16, 0, 0);
    result = ma_decoder_init_memory(caf.pData, caf.size, &config, &decoder);
    MA_ASSERT(result == MA_SUCCESS);

    result = ma_decoder_get_length_in_pcm_frames(&decoder, &length);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(length == g_testFrameCount);

    result = ma_decoder_get_cursor_in_pcm_frames(&decoder, &cursor);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(cursor == 0);

    /* Seek forward and confirm the cursor and the samples match a fresh sequential decode. */
    result = ma_decoder_seek_to_pcm_frame(&decoder, 10);
    MA_ASSERT(result == MA_SUCCESS);
    result = ma_decoder_get_cursor_in_pcm_frames(&decoder, &cursor);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(cursor == 10);

    result = ma_decoder_read_pcm_frames(&decoder, tmp, 5, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead == 5);
    MA_ASSERT((memcmp(tmp, g_s16Pattern + 10 * 2, (size_t)5 * 2 * sizeof(ma_int16)) == 0));

    result = ma_decoder_get_cursor_in_pcm_frames(&decoder, &cursor);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(cursor == 15);

    /* Seek back to zero and re-read the first frames. */
    result = ma_decoder_seek_to_pcm_frame(&decoder, 0);
    MA_ASSERT(result == MA_SUCCESS);
    result = ma_decoder_read_pcm_frames(&decoder, tmp, 3, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead == 3);
    MA_ASSERT((memcmp(tmp, g_s16Pattern, (size_t)3 * 2 * sizeof(ma_int16)) == 0));

    /*
    Seek past the end. The backend clamps the physical position to the last frame, so the very next
    read must immediately report end of stream with no frames. The decoder's logical cursor records
    the requested frame index (miniaudio's established convention, shared with the WAV backend).
    */
    result = ma_decoder_seek_to_pcm_frame(&decoder, length + 100);
    MA_ASSERT(result == MA_SUCCESS);
    result = ma_decoder_read_pcm_frames(&decoder, tmp, 4, &framesRead);
    MA_ASSERT(result == MA_AT_END);
    MA_ASSERT(framesRead == 0);

    ma_decoder_uninit(&decoder);
    test_buf_uninit(&caf);
    return MA_SUCCESS;
}

static ma_result test_caf__truncation_vs_unknown_codec(void)
{
    test_caf_layout lpcmLayout = { "s16 BE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
    test_buf truncatedCaf, unknownCodecCaf;
    ma_decoder_config config;
    ma_decoder decoder;
    ma_result result;
    ma_int16 tmp[64];
    ma_uint64 framesRead;

    /* 1) A structurally valid LPCM file whose data payload is cut in half. */
    test_build_caf(&truncatedCaf, &lpcmLayout, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount, MA_TRUE);
    config = ma_decoder_config_init(ma_format_s16, 0, 0);
    result = ma_decoder_init_memory(truncatedCaf.pData, truncatedCaf.size, &config, &decoder);
    MA_ASSERT(result == MA_SUCCESS);  /* Header parses; truncation only surfaces while reading. */

    /*
    A truncated payload surfaces as a short read: the first call returns the frames that are
    actually present (fewer than the declared length) and the following call, with no data left,
    returns MA_AT_END. This matches the miniaudio decoder convention shared by the WAV backend,
    where MA_AT_END is only reported when zero frames are delivered.
    */
    result = ma_decoder_read_pcm_frames(&decoder, tmp, g_testFrameCount, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead < g_testFrameCount);
    {
        ma_uint64 secondRead = 0;
        result = ma_decoder_read_pcm_frames(&decoder, tmp, g_testFrameCount, &secondRead);
        MA_ASSERT(result == MA_AT_END);
        MA_ASSERT(secondRead == 0);
    }
    ma_decoder_uninit(&decoder);
    test_buf_uninit(&truncatedCaf);

    /* 2) A well-formed CAF whose desc declares a codec we do not support. Build it by hand. */
    test_buf_init(&unknownCodecCaf);
    test_put_fourcc(&unknownCodecCaf, "caff");
    test_put_be16(&unknownCodecCaf, 1);
    test_put_be16(&unknownCodecCaf, 0);
    test_put_fourcc(&unknownCodecCaf, "desc");
    test_put_be64(&unknownCodecCaf, 32);
    test_put_fourcc(&unknownCodecCaf, "aac ");   /* Not 'lpcm'. */
    test_put_be32(&unknownCodecCaf, 0);
    test_put_be32(&unknownCodecCaf, 0);
    test_put_be32(&unknownCodecCaf, 1024);
    test_put_be32(&unknownCodecCaf, 2);
    test_put_be32(&unknownCodecCaf, 16);
    test_put_be_f64(&unknownCodecCaf, 44100.0);

    result = ma_decoder_init_memory(unknownCodecCaf.pData, unknownCodecCaf.size, &config, &decoder);
    MA_ASSERT(result == MA_NOT_IMPLEMENTED);  /* Distinct from MA_INVALID_FILE / MA_AT_END. */
    test_buf_uninit(&unknownCodecCaf);

    /* 3) Garbage that is not a CAF at all must be rejected as an invalid file (and not crash). */
    {
        static const ma_uint8 junk[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
        result = ma_decoder_init_memory(junk, sizeof(junk), &config, &decoder);
        MA_ASSERT(result != MA_SUCCESS);
        MA_ASSERT(result != MA_NOT_IMPLEMENTED);
    }

    /* 4) A correct magic followed by a truncated header must fail cleanly rather than over-read. */
    {
        static const ma_uint8 truncatedMagic[10] = { 'c','a','f','f', 0,1, 0,0, 'd','e' };
        result = ma_decoder_init_memory(truncatedMagic, sizeof(truncatedMagic), &config, &decoder);
        MA_ASSERT(result != MA_SUCCESS);
    }

    /* 5) A caff file with no desc and no data chunk is structurally invalid. */
    {
        test_buf empty;
        test_buf_init(&empty);
        test_put_fourcc(&empty, "caff");
        test_put_be16(&empty, 1);
        test_put_be16(&empty, 0);
        result = ma_decoder_init_memory(empty.pData, empty.size, &config, &decoder);
        MA_ASSERT(result == MA_INVALID_FILE);
        test_buf_uninit(&empty);
    }

    /* 6) A zero-length buffer must be rejected. */
    {
        static const ma_uint8 one = 0;
        result = ma_decoder_init_memory(&one, 0, &config, &decoder);
        MA_ASSERT(result == MA_INVALID_ARGS);
    }

    return MA_SUCCESS;
}

static ma_result test_caf__container_detection(void)
{
    /* A CAF with a non-.caf name is still found by content sniffing; explicit caf format also works. */
    test_caf_layout layout = { "s16 LE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH, MA_TRUE };
    test_buf caf;
    ma_decoder_config config;
    ma_decoder decoder;
    ma_result result;
    ma_int16 tmp[64];
    ma_uint64 framesRead;
    ma_uint32 channels = 0, rate = 0;

    test_build_caf(&caf, &layout, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount, MA_FALSE);

    /* Generic init with unknown encoding format relies on content sniffing. */
    config = ma_decoder_config_init(ma_format_s16, 0, 0);
    result = ma_decoder_init_memory(caf.pData, caf.size, &config, &decoder);
    MA_ASSERT(result == MA_SUCCESS);
    result = ma_decoder_get_data_format(&decoder, NULL, &channels, &rate, NULL, 0);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(channels == 2);
    MA_ASSERT(rate == 44100);
    result = ma_decoder_read_pcm_frames(&decoder, tmp, g_testFrameCount, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead == g_testFrameCount);
    MA_ASSERT((memcmp(tmp, g_s16Pattern, sizeof(g_s16Pattern)) == 0));
    ma_decoder_uninit(&decoder);

    /* Explicit caf encoding format. */
    config = ma_decoder_config_init(ma_format_s16, 0, 0);
    config.encodingFormat = ma_encoding_format_caf;
    result = ma_decoder_init_memory(caf.pData, caf.size, &config, &decoder);
    MA_ASSERT(result == MA_SUCCESS);
    ma_decoder_uninit(&decoder);

    test_buf_uninit(&caf);
    return MA_SUCCESS;
}

static ma_result test_caf__length_without_pakt(void)
{
    /* With no packet table, the length must be derived from the data chunk size. */
    test_caf_layout layout = { "s16 LE no-pakt", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH, MA_FALSE };
    test_buf caf;
    ma_decoder_config config;
    ma_decoder decoder;
    ma_result result;
    ma_uint64 length = 0;
    ma_int16 tmp[64];
    ma_uint64 framesRead;

    test_build_caf(&caf, &layout, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount, MA_FALSE);

    config = ma_decoder_config_init(ma_format_s16, 0, 0);
    result = ma_decoder_init_memory(caf.pData, caf.size, &config, &decoder);
    MA_ASSERT(result == MA_SUCCESS);

    result = ma_decoder_get_length_in_pcm_frames(&decoder, &length);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(length == g_testFrameCount);  /* Derived from data size / bytes-per-frame. */

    result = ma_decoder_read_pcm_frames(&decoder, tmp, g_testFrameCount, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead == g_testFrameCount);
    MA_ASSERT((memcmp(tmp, g_s16Pattern, sizeof(g_s16Pattern)) == 0));

    ma_decoder_uninit(&decoder);
    test_buf_uninit(&caf);
    return MA_SUCCESS;
}


typedef struct
{
    const ma_uint8* pData;
    size_t size;
    size_t cursor;
} test_caf_stream;

static ma_result test_stream_on_read(ma_decoder* pDecoder, void* pOut, size_t bytesToRead, size_t* pBytesRead)
{
    test_caf_stream* pS = (test_caf_stream*)pDecoder->pUserData;
    size_t remaining = pS->size - pS->cursor;
    (void)pDecoder;

    if (bytesToRead > remaining) {
        bytesToRead = remaining;
    }
    if (bytesToRead > 0) {
        MA_COPY_MEMORY(pOut, pS->pData + pS->cursor, bytesToRead);
        pS->cursor += bytesToRead;
    }
    if (pBytesRead != NULL) {
        *pBytesRead = bytesToRead;
    }
    return bytesToRead == 0 ? MA_AT_END : MA_SUCCESS;
}

static ma_result test_stream_on_seek(ma_decoder* pDecoder, ma_int64 offset, ma_seek_origin origin)
{
    test_caf_stream* pS = (test_caf_stream*)pDecoder->pUserData;
    ma_int64 target;
    (void)pDecoder;

    if (origin == ma_seek_origin_start) {
        target = offset;
    } else if (origin == ma_seek_origin_current) {
        target = (ma_int64)pS->cursor + offset;
    } else {
        target = (ma_int64)pS->size + offset;
    }
    if (target < 0) {
        target = 0;
    }
    if ((size_t)target > pS->size) {
        target = (ma_int64)pS->size;
    }
    pS->cursor = (size_t)target;
    return MA_SUCCESS;
}

static ma_result test_caf__callback_no_tell(void)
{
    /* A callback stream that supplies read/seek but no tell must still parse and decode fully. */
    test_caf_layout layout = { "s16 BE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
    test_buf caf;
    test_caf_stream stream;
    ma_decoder_config config;
    ma_decoder decoder;
    ma_result result;
    ma_int16 tmp[64];
    ma_uint64 framesRead = 0;

    test_build_caf(&caf, &layout, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount, MA_FALSE);
    stream.pData = caf.pData;
    stream.size = caf.size;
    stream.cursor = 0;

    config = ma_decoder_config_init(ma_format_s16, 0, 0);
    result = ma_decoder_init(test_stream_on_read, test_stream_on_seek, &stream, &config, &decoder);
    MA_ASSERT(result == MA_SUCCESS);

    result = ma_decoder_read_pcm_frames(&decoder, tmp, g_testFrameCount, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead == g_testFrameCount);
    MA_ASSERT((memcmp(tmp, g_s16Pattern, sizeof(g_s16Pattern)) == 0));

    /* Seeking also works without a tell callback because the backend tracks its own frame cursor. */
    result = ma_decoder_seek_to_pcm_frame(&decoder, 5);
    MA_ASSERT(result == MA_SUCCESS);
    result = ma_decoder_read_pcm_frames(&decoder, tmp, 2, &framesRead);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(framesRead == 2);
    MA_ASSERT((memcmp(tmp, g_s16Pattern + 5 * 2, (size_t)2 * 2 * sizeof(ma_int16)) == 0));

    ma_decoder_uninit(&decoder);
    test_buf_uninit(&caf);
    return MA_SUCCESS;
}



static ma_uint32 g_liveBytes;
static ma_uint32 g_peakBytes;

static void* test_aligned_malloc(size_t sz, void* pUserData)
{
    ma_uint8* p;
    (void)pUserData;
    p = (ma_uint8*)ma_malloc(sz + sizeof(size_t), NULL);
    MA_ASSERT(p != NULL);
    MA_COPY_MEMORY(p, &sz, sizeof(sz));
    g_liveBytes += (ma_uint32)sz;
    if (g_liveBytes > g_peakBytes) {
        g_peakBytes = g_liveBytes;
    }
    return p + sizeof(size_t);
}

static void test_aligned_free(void* p, void* pUserData)
{
    size_t sz;
    ma_uint8* pBase;
    (void)pUserData;
    if (p == NULL) {
        return;
    }
    pBase = (ma_uint8*)p - sizeof(size_t);
    MA_COPY_MEMORY(&sz, pBase, sizeof(sz));
    g_liveBytes -= (ma_uint32)sz;
    ma_free(pBase, NULL);
}

static void* test_aligned_realloc(void* p, size_t sz, void* pUserData)
{
    void* pNew;
    size_t oldSz;
    ma_uint8* pBase;

    if (p == NULL) {
        return test_aligned_malloc(sz, pUserData);
    }

    pBase = (ma_uint8*)p - sizeof(size_t);
    MA_COPY_MEMORY(&oldSz, pBase, sizeof(oldSz));

    pNew = test_aligned_malloc(sz, pUserData);
    MA_COPY_MEMORY(pNew, p, oldSz < sz ? oldSz : sz);
    test_aligned_free(p, pUserData);
    return pNew;
}

static ma_result test_caf__constant_peak_memory(void)
{
    test_caf_layout layout = { "s24 BE packed", ma_format_s24, 24, 3, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_BIG_ENDIAN, MA_TRUE };
    test_buf smallCaf, bigCaf;
    ma_allocation_callbacks callbacks;
    ma_decoder_config config;
    ma_decoder smallDec, bigDec;
    ma_result result;
    ma_uint32 peakSmall, peakBig;
    ma_uint8* pSamples;
    size_t sampleBytes;
    ma_uint64 frameCount;
    ma_uint64 framesRead;
    ma_uint8 chunk[512];

    /* A small file and a ~64 KB-sample file. */
    frameCount = 64;
    sampleBytes = (size_t)frameCount * 2 * 3;
    pSamples = (ma_uint8*)ma_malloc(sampleBytes, NULL);
    {
        size_t i;
        for (i = 0; i < sampleBytes; i += 1) {
            pSamples[i] = (ma_uint8)(i * 7 + 1);
        }
    }
    test_build_caf(&smallCaf, &layout, 2, 48000, pSamples, sampleBytes, frameCount, MA_FALSE);

    frameCount = 64 * 1024;
    sampleBytes = (size_t)frameCount * 2 * 3;
    ma_free(pSamples, NULL);
    pSamples = (ma_uint8*)ma_malloc(sampleBytes, NULL);
    {
        size_t i;
        for (i = 0; i < sampleBytes; i += 1) {
            pSamples[i] = (ma_uint8)(i * 13 + 5);
        }
    }
    test_build_caf(&bigCaf, &layout, 2, 48000, pSamples, sampleBytes, frameCount, MA_FALSE);

    callbacks = ma_allocation_callbacks_init_default();
    callbacks.onMalloc = test_aligned_malloc;
    callbacks.onRealloc = test_aligned_realloc;
    callbacks.onFree = test_aligned_free;

    config = ma_decoder_config_init(ma_format_s24, 0, 0);
    config.allocationCallbacks = callbacks;

    g_liveBytes = 0;
    g_peakBytes = 0;
    result = ma_decoder_init_memory(smallCaf.pData, smallCaf.size, &config, &smallDec);
    MA_ASSERT(result == MA_SUCCESS);
    for (;;) {
        result = ma_decoder_read_pcm_frames(&smallDec, chunk, 32, &framesRead);
        if (result == MA_AT_END) {
            break;
        }
        MA_ASSERT(result == MA_SUCCESS);
    }
    peakSmall = g_peakBytes;
    ma_decoder_uninit(&smallDec);
    MA_ASSERT(g_liveBytes == 0);

    g_liveBytes = 0;
    g_peakBytes = 0;
    result = ma_decoder_init_memory(bigCaf.pData, bigCaf.size, &config, &bigDec);
    MA_ASSERT(result == MA_SUCCESS);
    for (;;) {
        result = ma_decoder_read_pcm_frames(&bigDec, chunk, 32, &framesRead);
        if (result == MA_AT_END) {
            break;
        }
        MA_ASSERT(result == MA_SUCCESS);
    }
    peakBig = g_peakBytes;
    ma_decoder_uninit(&bigDec);
    MA_ASSERT(g_liveBytes == 0);

    /* The 1024x larger file must not inflate the decoder's peak working memory. */
    MA_ASSERT(peakBig <= peakSmall + 64);

    ma_free(pSamples, NULL);
    test_buf_uninit(&smallCaf);
    test_buf_uninit(&bigCaf);
    return MA_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* No-regression: WAV/MP3/FLAC still dispatch and decode as before.   */
/* ------------------------------------------------------------------ */

static ma_result test_caf__existing_formats_unaffected(void)
{
    /* A plain WAV must still decode through the WAV backend with the same samples. */
    test_buf wav;
    ma_int16 out[256];
    ma_uint64 frames = 0;
    ma_result result;

    test_build_wav(&wav, ma_format_s16, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount);
    result = test_decode_all(wav.pData, wav.size, ma_format_s16, out, 256, NULL, NULL, &frames, NULL);
    MA_ASSERT(result == MA_SUCCESS);
    MA_ASSERT(frames == g_testFrameCount);
    MA_ASSERT((memcmp(out, g_s16Pattern, sizeof(g_s16Pattern)) == 0));
    test_buf_uninit(&wav);

    /* An explicitly-requested WAV decode of CAF bytes must fail rather than mis-handle them. */
    {
        test_caf_layout layout = { "s16 LE", ma_format_s16, 16, 2, TEST_FLAG_SIGNED_INT | TEST_FLAG_PACKED | TEST_FLAG_ALIGNED_HIGH, MA_TRUE };
        test_buf caf;
        ma_decoder_config config = ma_decoder_config_init(ma_format_s16, 0, 0);
        ma_decoder decoder;
        config.encodingFormat = ma_encoding_format_wav;
        test_build_caf(&caf, &layout, 2, 44100, g_s16Pattern, sizeof(g_s16Pattern), g_testFrameCount, MA_FALSE);
        result = ma_decoder_init_memory(caf.pData, caf.size, &config, &decoder);
        MA_ASSERT(result != MA_SUCCESS);
        test_buf_uninit(&caf);
    }

    return MA_SUCCESS;
}

int test_entry__caf(int argc, char** argv)
{
    ma_result result;
    ma_bool32 hasError = MA_FALSE;

    (void)argc;
    (void)argv;

    result = test_caf__s16_endian();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__s24_packed();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__formats_via_f32();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__length_and_cursor();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__truncation_vs_unknown_codec();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__container_detection();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__length_without_pakt();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__callback_no_tell();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__constant_peak_memory();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    result = test_caf__existing_formats_unaffected();
    if (result != MA_SUCCESS) { hasError = MA_TRUE; }

    return hasError ? -1 : 0;
}

int main(int argc, char** argv)
{
    ma_register_test("CAF Decoder", test_entry__caf);
    return ma_run_tests(argc, argv);
}
