/**
 * PC-port audio stubs — Phase-4 deferral of the AL synthesizer.
 *
 * The real libultra audio library (src/libultra/audio + src/libultrare) is
 * scheduled for Phase 4. Until then every al* symbol the kept game code
 * references is satisfied here so the game runs SILENT without crashing.
 *
 * Design rules used below (each stub carries its own rationale):
 *  - Functions whose results feed loops/derefs get REAL minimal
 *    implementations (alLink/alUnlink, alHeapInit/alHeapDBAlloc).
 *  - Functions that only drive the (never-running) synthesizer are no-ops.
 *  - Query functions return the value that makes every audited caller take
 *    its benign exit path (alCSPGetState -> AL_STOPPED, alEvtqNextEvent ->
 *    positive delta, alCents2Ratio -> 1.0).
 *  - The SFX sound-bank consumed by snd.c sndPlaySfx() cannot be stubbed
 *    with zeros (it is deref'd 3 levels deep before any al* call), so a
 *    minimal fake bank graph is provided (portPcmBank below) and wired in
 *    via the crafted _sfxctl/_instrumentsctl segment blobs in
 *    port/src/segments.c.
 *
 * Callers audited: src/audi.c (amCreateAudioManager/amHandleFrameMessage),
 * src/music.c (musicSeqPlayerInit/musicTrackXPlay/Stop/ApplySeqpVol),
 * src/snd.c (sndNewPlayerInit/sndPlaySfx/sndSetupSound/sndPlayerVoiceHandler).
 */
#include <string.h>
#include <stdint.h>

#include <ultra64.h>
#include <PR/libaudio.h>

/* ==== AUDIO PHASE 1c ===================================================== *
 * The REAL AL library is now compiled (see CMakeLists' audio block): sl.c
 * provides alGlobals/alInit/alClose/alLink/alUnlink; heapinit/heapalloc
 * the heap; event.c the event queues; synthesizer.c + syn*.c the voices;
 * load/resample/env_wip/save/buses/reverb/drvrNew the filter chain.
 * port/src/audio_pc.c executes the emitted Acmd lists and feeds SDL.
 * The stubs that remain below are ONLY the compressed-MIDI sequencer
 * layer (music phase) and the bank loader (portBnkfLoad, which replaces
 * alBnkfNew on PC). */

/* ==== bank file ========================================================== */
/*
 * portBnkfLoad builds host-layout bank structs from the big-endian SGI .ctl
 * image the ROM serves (revision 'B1'). bnkf.c on N64 patches the image in
 * place, turning 4-byte offsets into 4-byte pointers; that only holds while a
 * pointer is 4 bytes, so PC reads the image and writes fresh structs instead,
 * the same code at either width. Shape follows the PD port's
 * port/src/preprocess/segaudio.c from its 64-bit lift.
 *
 * .ctl layout (offsets from the file start, all big-endian):
 *   bankfile   s16 revision, s16 bankCount, u32 bank[]
 *   bank       s16 instCount, u8 flags, u8 pad, s32 rate, u32 percussion, u32 inst[]
 *   instrument 12 x u8, s16 bendRange, s16 soundCount, u32 sound[]
 *   sound      u32 envelope, u32 keymap, u32 wavetable, u8 pan, u8 vol, u8 flags
 *   wavetable  u32 base (.tbl offset), s32 len, u8 type, u8 flags, u32 loop, u32 book
 *   envelope   s32 attack, s32 decay, s32 release, u8 attackVol, u8 decayVol
 *   keymap     6 x u8
 *   adpcm loop u32 start, u32 end, u32 count, s16 state[16]
 *   adpcm book s32 order, s32 npredictors, s16 book[order * npredictors * 8]
 *   raw loop   u32 start, u32 end, u32 count
 *
 * Records can be shared between parents, so each .ctl offset is converted
 * once and later references reuse the host copy. bank / instrument / sound /
 * wavetable flags are set to 1 as SGI's walk does (its revisit guard), and a
 * zero bank or instrument entry points at the file as it did before. RAW16
 * samples in the .tbl are swapped to native s16 once per wave (the mixer
 * plays native samples; ADPCM decodes bytes). The host image is never freed:
 * banks live for the whole run.
 */
#include <stdio.h>
#include <stdlib.h>

extern void portConLogf(const char *fmt, ...);

#define BNKF_SEEN_MAX 8192

static struct { u32 src; void *host; } s_bnkfSeen[BNKF_SEEN_MAX];
static s32 s_bnkfSeenCount;
static const void *s_bnkfWaveSeen[BNKF_SEEN_MAX];
static s32 s_bnkfWaveSeenCount;

static const u8 *s_bnkfSrc;
static u32 s_bnkfSrcSize;
static u8 *s_bnkfDst;
static u32 s_bnkfDstPos;
static u32 s_bnkfDstSize;
static u8 *s_bnkfTable;
static ALBankFile *s_bnkfFile;
static s32 s_bnkfNullRefs;

static u16 portBnkfBe16(const u8 *p)
{
    return (u16)((p[0] << 8) | p[1]);
}

static u32 portBnkfBe32(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static void portBnkfFail(const char *what, u32 off)
{
    portConLogf("ge: bnkf: %s at .ctl offset 0x%x - the audio bank cannot be loaded\n", what, off);
    exit(1);
}

/* the source record at `off`, bounds-checked */
static const u8 *portBnkfAt(u32 off, u32 size)
{
    if (off > s_bnkfSrcSize || size > s_bnkfSrcSize - off) {
        portBnkfFail("record runs past the end of the file", off);
    }
    return s_bnkfSrc + off;
}

/* the next host record, aligned for the pointers it holds */
static void *portBnkfAlloc(u32 size)
{
    void *p;

    s_bnkfDstPos = (s_bnkfDstPos + 7u) & ~7u;
    if (size > s_bnkfDstSize - s_bnkfDstPos) {
        portBnkfFail("host image too small", s_bnkfDstPos);
    }
    p = s_bnkfDst + s_bnkfDstPos;
    s_bnkfDstPos += size;
    return p;
}

static void *portBnkfSeen(u32 off)
{
    s32 i;

    for (i = 0; i < s_bnkfSeenCount; i++) {
        if (s_bnkfSeen[i].src == off) {
            return s_bnkfSeen[i].host;
        }
    }
    return NULL;
}

static void portBnkfRemember(u32 off, void *host)
{
    if (s_bnkfSeenCount >= BNKF_SEEN_MAX) {
        portBnkfFail("more records than BNKF_SEEN_MAX", off);
    }
    s_bnkfSeen[s_bnkfSeenCount].src = off;
    s_bnkfSeen[s_bnkfSeenCount].host = host;
    s_bnkfSeenCount++;
}

/* elements past the [1] a variable-length struct already declares */
static u32 portBnkfExtra(s32 count, u32 elemsize)
{
    return count > 1 ? (u32)(count - 1) * elemsize : 0;
}

static ALEnvelope *portBnkfEnvelope(u32 off)
{
    const u8 *s;
    ALEnvelope *e;

    if (off == 0) {
        s_bnkfNullRefs++;
        return NULL;
    }
    if ((e = portBnkfSeen(off)) != NULL) {
        return e;
    }
    s = portBnkfAt(off, 14);
    e = portBnkfAlloc(sizeof(ALEnvelope));
    portBnkfRemember(off, e);
    e->attackTime = (s32)portBnkfBe32(s + 0);
    e->decayTime = (s32)portBnkfBe32(s + 4);
    e->releaseTime = (s32)portBnkfBe32(s + 8);
    e->attackVolume = s[12];
    e->decayVolume = s[13];
    return e;
}

static ALKeyMap *portBnkfKeyMap(u32 off)
{
    const u8 *s;
    ALKeyMap *k;

    if (off == 0) {
        s_bnkfNullRefs++;
        return NULL;
    }
    if ((k = portBnkfSeen(off)) != NULL) {
        return k;
    }
    s = portBnkfAt(off, 6);
    k = portBnkfAlloc(sizeof(ALKeyMap));
    portBnkfRemember(off, k);
    k->velocityMin = s[0];
    k->velocityMax = s[1];
    k->keyMin = s[2];
    k->keyMax = s[3];
    k->keyBase = s[4];
    k->detune = (s8)s[5];
    return k;
}

static ALADPCMloop *portBnkfAdpcmLoop(u32 off)
{
    const u8 *s;
    ALADPCMloop *l;
    s32 n;

    if ((l = portBnkfSeen(off)) != NULL) {
        return l;
    }
    s = portBnkfAt(off, 12 + 16 * 2);
    l = portBnkfAlloc(sizeof(ALADPCMloop));
    portBnkfRemember(off, l);
    l->start = portBnkfBe32(s + 0);
    l->end = portBnkfBe32(s + 4);
    l->count = portBnkfBe32(s + 8);
    for (n = 0; n < 16; n++) {
        ((s16 *)l->state)[n] = (s16)portBnkfBe16(s + 12 + n * 2);
    }
    return l;
}

static ALADPCMBook *portBnkfAdpcmBook(u32 off)
{
    const u8 *s;
    ALADPCMBook *b;
    s32 order, npred, n, ncoef;

    if ((b = portBnkfSeen(off)) != NULL) {
        return b;
    }
    s = portBnkfAt(off, 8);
    order = (s32)portBnkfBe32(s + 0);
    npred = (s32)portBnkfBe32(s + 4);
    if (order < 0 || npred < 0 || order > 16 || npred > 16) {
        portBnkfFail("implausible ADPCM book size", off);
    }
    ncoef = order * npred * 8;
    s = portBnkfAt(off, 8 + (u32)ncoef * 2);
    b = portBnkfAlloc(sizeof(ALADPCMBook) + portBnkfExtra(ncoef, sizeof(s16)));
    portBnkfRemember(off, b);
    b->order = order;
    b->npredictors = npred;
    for (n = 0; n < ncoef; n++) {
        b->book[n] = (s16)portBnkfBe16(s + 8 + n * 2);
    }
    return b;
}

static ALRawLoop *portBnkfRawLoop(u32 off)
{
    const u8 *s;
    ALRawLoop *l;

    if ((l = portBnkfSeen(off)) != NULL) {
        return l;
    }
    s = portBnkfAt(off, 12);
    l = portBnkfAlloc(sizeof(ALRawLoop));
    portBnkfRemember(off, l);
    l->start = portBnkfBe32(s + 0);
    l->end = portBnkfBe32(s + 4);
    l->count = portBnkfBe32(s + 8);
    return l;
}

/* RAW16 sample data: big-endian in the .tbl, native for the mixer; swap once */
static void portBnkfSwapRawSamples(ALWaveTable *w)
{
    s16 *smp = (s16 *)w->base;
    s32 ns = w->len >> 1;
    s32 i;

    for (i = 0; i < s_bnkfWaveSeenCount; i++) {
        if (s_bnkfWaveSeen[i] == w->base) {
            return;
        }
    }
    if (s_bnkfWaveSeenCount < BNKF_SEEN_MAX) {
        s_bnkfWaveSeen[s_bnkfWaveSeenCount++] = w->base;
    }
    for (i = 0; i < ns; i++) {
        u16 v = (u16)smp[i];
        smp[i] = (s16)((v >> 8) | (v << 8));
    }
}

static ALWaveTable *portBnkfWaveTable(u32 off)
{
    const u8 *s;
    ALWaveTable *w;
    u32 loop, book;

    if (off == 0) {
        s_bnkfNullRefs++;
        return NULL;
    }
    if ((w = portBnkfSeen(off)) != NULL) {
        return w;
    }
    s = portBnkfAt(off, 20);
    w = portBnkfAlloc(sizeof(ALWaveTable));
    portBnkfRemember(off, w);
    w->base = s_bnkfTable + portBnkfBe32(s + 0);
    w->len = (s32)portBnkfBe32(s + 4);
    w->type = s[8];
    w->flags = 1;
    loop = portBnkfBe32(s + 12);
    book = portBnkfBe32(s + 16);

    if (w->type == AL_ADPCM_WAVE) {
        w->waveInfo.adpcmWave.loop = loop ? portBnkfAdpcmLoop(loop) : NULL;
        w->waveInfo.adpcmWave.book = book ? portBnkfAdpcmBook(book) : NULL;
    } else if (w->type == AL_RAW16_WAVE) {
        w->waveInfo.rawWave.loop = loop ? portBnkfRawLoop(loop) : NULL;
        portBnkfSwapRawSamples(w);
    }
    return w;
}

static ALSound *portBnkfSound(u32 off)
{
    const u8 *s;
    ALSound *snd;

    if (off == 0) {
        s_bnkfNullRefs++;
        return NULL;
    }
    if ((snd = portBnkfSeen(off)) != NULL) {
        return snd;
    }
    s = portBnkfAt(off, 15);
    snd = portBnkfAlloc(sizeof(ALSound));
    portBnkfRemember(off, snd);
    snd->envelope = portBnkfEnvelope(portBnkfBe32(s + 0));
    snd->keyMap = portBnkfKeyMap(portBnkfBe32(s + 4));
    snd->wavetable = portBnkfWaveTable(portBnkfBe32(s + 8));
    snd->samplePan = s[12];
    snd->sampleVolume = s[13];
    snd->flags = 1;
    return snd;
}

static ALInstrument *portBnkfInstrument(u32 off)
{
    const u8 *s;
    ALInstrument *inst;
    s16 n;
    s32 i;

    if (off == 0) {
        return (ALInstrument *)s_bnkfFile;
    }
    if ((inst = portBnkfSeen(off)) != NULL) {
        return inst;
    }
    s = portBnkfAt(off, 16);
    n = (s16)portBnkfBe16(s + 14);
    if (n < 0) {
        portBnkfFail("negative sound count", off);
    }
    s = portBnkfAt(off, 16 + (u32)n * 4);
    inst = portBnkfAlloc(sizeof(ALInstrument) + portBnkfExtra(n, sizeof(ALSound *)));
    portBnkfRemember(off, inst);
    inst->volume = s[0];
    inst->pan = s[1];
    inst->priority = s[2];
    inst->flags = 1;
    inst->tremType = s[4];
    inst->tremRate = s[5];
    inst->tremDepth = s[6];
    inst->tremDelay = s[7];
    inst->vibType = s[8];
    inst->vibRate = s[9];
    inst->vibDepth = s[10];
    inst->vibDelay = s[11];
    inst->bendRange = (s16)portBnkfBe16(s + 12);
    inst->soundCount = n;
    for (i = 0; i < n; i++) {
        inst->soundArray[i] = portBnkfSound(portBnkfBe32(s + 16 + i * 4));
    }
    return inst;
}

static ALBank *portBnkfBank(u32 off)
{
    const u8 *s;
    ALBank *b;
    s16 n;
    s32 i;
    u32 perc;

    if (off == 0) {
        return (ALBank *)s_bnkfFile;
    }
    if ((b = portBnkfSeen(off)) != NULL) {
        return b;
    }
    s = portBnkfAt(off, 12);
    n = (s16)portBnkfBe16(s + 0);
    if (n < 0) {
        portBnkfFail("negative instrument count", off);
    }
    s = portBnkfAt(off, 12 + (u32)n * 4);
    b = portBnkfAlloc(sizeof(ALBank) + portBnkfExtra(n, sizeof(ALInstrument *)));
    portBnkfRemember(off, b);
    b->instCount = n;
    b->flags = 1;
    b->pad = s[3];
    b->sampleRate = (s32)portBnkfBe32(s + 4);
    perc = portBnkfBe32(s + 8);
    b->percussion = perc ? portBnkfInstrument(perc) : NULL;
    for (i = 0; i < n; i++) {
        b->instArray[i] = portBnkfInstrument(portBnkfBe32(s + 12 + i * 4));
    }
    return b;
}

/**
 * Build a host ALBankFile from the big-endian .ctl image at `ctl` (`size`
 * bytes, read-only), with wave bases resolved against `table` (the .tbl).
 * Replaces romCopy + alBnkfNew on PC (src/music.c).
 */
ALBankFile *portBnkfLoad(const void *ctl, u32 size, u8 *table)
{
    const u8 *s = ctl;
    ALBankFile *f;
    s16 count;
    s32 i;

    if (size < 4 || portBnkfBe16(s) != AL_BANK_VERSION) {
        portConLogf("ge: bnkf: not a big-endian B1 bank file (%u bytes) - the audio bank cannot be loaded\n", size);
        exit(1);
    }

    s_bnkfSrc = s;
    s_bnkfSrcSize = size;
    s_bnkfTable = table;
    s_bnkfSeenCount = 0;
    s_bnkfWaveSeenCount = 0;
    s_bnkfNullRefs = 0;
    /* 8-byte pointers over 4-byte offsets, plus alignment: under 3x in
     * practice, and portBnkfAlloc stops loudly if a bank ever needs more */
    s_bnkfDstSize = size * 4;
    s_bnkfDstPos = 0;
    s_bnkfDst = calloc(1, s_bnkfDstSize);
    if (s_bnkfDst == NULL) {
        portBnkfFail("out of memory", 0);
    }

    count = (s16)portBnkfBe16(s + 2);
    if (count < 1) {
        portBnkfFail("no banks", 2);
    }
    portBnkfAt(0, 4 + (u32)count * 4);
    f = portBnkfAlloc(sizeof(ALBankFile) + portBnkfExtra(count, sizeof(ALBank *)));
    s_bnkfFile = f;
    f->revision = AL_BANK_VERSION;
    f->bankCount = count;
    for (i = 0; i < count; i++) {
        f->bankArray[i] = portBnkfBank(portBnkfBe32(s + 4 + i * 4));
    }

    portConLogf("ge: bnkf: %u-byte .ctl -> %u bytes host, %d records%s\n",
            size, s_bnkfDstPos, s_bnkfSeenCount,
            s_bnkfNullRefs ? " (some zero references left NULL)" : "");

    { /* bring-up (GE_DUMP_BOND): prove the parse - bank/inst/sound shape */
        extern void ge_dbg_vec(const char *tag, const float *v, int n);
        ALBank *bank = f->bankArray[0];
        float v[7];

        v[0] = (float)f->bankCount;
        v[1] = (float)bank->instCount;
        v[2] = (float)bank->sampleRate;
        v[3] = (float)bank->instArray[0]->soundCount;
        v[4] = (float)bank->instArray[0]->soundArray[0]->wavetable->type;
        v[5] = (float)bank->instArray[0]->soundArray[0]->wavetable->len;
        v[6] = (float)(uintptr_t)bank->instArray[0]->soundArray[0]->wavetable->base;
        ge_dbg_vec("bnkf banks/insts/rate/sounds/wt0type/len/base", v, 7);
    }

    return f;
}

/* ==== compressed-midi seq / seq player =================================== */

/**
 * Real alCSeqNew parses a compressed MIDI stream header. The stream our
 * crafted sample table produces is an empty deflate block (decompresses
 * to 0 bytes), so there is nothing to parse; zero the struct so any later
 * reader sees a consistent "empty sequence".
 */
/* (The compressed-MIDI sequencer stubs - alCSeqNew, the alCSP set, and
 * alSeqpSetBank - were retired with the MUSIC PHASE: cseq.c, csplayer.c,
 * the csp wrappers and seqpsetbank.c are compiled for real now, and the
 * real music.sbk feeds them via _musicsampletblSegmentRomStart.) */

/* (alCents2Ratio, alAudioFrame, the alSyn* voice API, and the fake
 * portPcmBank graph were retired with AUDIO PHASE 1c: the real library
 * provides the former, and the real banks replaced the latter.) */
