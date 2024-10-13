#ifndef __WAV_H__
#define __WAV_H__

#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <pthread.h>

/* PCMデフォルト設定 */
#define PCM_DEVICE          "default"
#define SOFT_RESAMPLE       1
#define LATENCY             50000
#define WAVE_FORMAT_ID      1
#define DEF_CHANNELS        2
#define DEF_SAMPLING_RATE   48000
#define DEF_BITPERSAMPLE    16
#define BUF_SAMPLES         1

/* ヘッダー構造体 */
typedef struct {
    char            riff[4];          // RIFFヘッダ
    unsigned int    fileSize;         // ファイルサイズ - 8
    char            wave[4];          // WAVEヘッダ
} wavHeader;

/* データチャンク構造体 */
typedef struct {
    unsigned char   fmt[4];           // fmt チャンク
    int             fmtSize;          // fmt チャンクのバイト数
} tagChank;

/* データフォーマット構造体 */
typedef struct {
    unsigned short  id;               // フォーマットID
    unsigned short  channels;         // チャンネル数
    unsigned int    samplingRate;     // サンプリングレート
    unsigned int    bytesPerSec;      // データ速度 (Byte/sec)
    unsigned short  blockSize;        // ブロックサイズ
    unsigned short  bitsPerSample;    // サンプルあたりのビット数
} wavFormat;

/* void playWAV(const char *fileName) { */
void *playWAV(void *fileNamePtr) {
    const char *fileName = (const char *)fileNamePtr;
    FILE *fp;
    wavHeader header;
    tagChank chank;
    wavFormat wf;
    int buffSize;
    char *pcmBuffer = NULL;
    snd_pcm_t *handle = NULL;
    static snd_pcm_format_t format = SND_PCM_FORMAT_S16;  /* 符号付き16bit */
    
    if ((fp = fopen(fileName, "rb")) == NULL) {
        printf("ファイルが開けません\n");
        return 0;
    }

    /* ヘッダー情報の読み取り */
    fread(&header, sizeof(wavHeader), 1, fp);
    header.riff[4] = '\0';
    header.wave[4] = '\0';
    /* printf("#識別子             : %s\n", header.riff); */
    /* printf("#ファイルサイズ     : %d[bytes]\n", header.fileSize + 8); */
    /* printf("#ファイル形式       : %s\n", header.wave); */

    /* チャンクの読み取り */
    fread(&chank, sizeof(chank), 1, fp);
    /* long len = chank.fmtSize; */
    chank.fmt[4] = '\0';
    /* printf("#fmt                : %s\n", chank.fmt); */
    /* printf("#fmtチャンクサイズ  : %ld[bytes]\n", len); */

    /* 各種フォーマットデータの読み取り */
    fread(&wf, sizeof(wavFormat), 1, fp);
    /* printf("#format ID(PCM=1)   : %d (0x%04x)\n", wf.id, wf.id); */
    /* printf("#チャンネル数       : %d (モノラル=1 ステレオ=2)\n", wf.channels); */
    /* printf("#サンプリングレート : %d[Hz]\n", wf.samplingRate); */
    /* printf("#データ速度         : %d[bytes/sec]\n", wf.bytesPerSec); */
    /* printf("#ブロックサイズ     : %d[bytes]\n", wf.blockSize); */
    /* printf("#量子化ビット数     : %d[bit]\n", wf.bitsPerSample); */
    /* printf("#再生時間           : %.2f[sec]\n", (double)(header.fileSize + 8) / wf.bytesPerSec); */

    /* 再生設定 */
    buffSize = BUF_SAMPLES * (wf.bitsPerSample / 8) * wf.channels;
    pcmBuffer = (char *)malloc(buffSize);
    
    /* PCMデバイスのオープン */
    if (snd_pcm_open(&handle, PCM_DEVICE, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        printf("PCMデバイスを開けません\n");
        free(pcmBuffer);
        fclose(fp);
        return 0;
    }

    /* PCMパラメーターの設定 */
    snd_pcm_set_params(handle, format, SND_PCM_ACCESS_RW_INTERLEAVED, wf.channels, wf.samplingRate, SOFT_RESAMPLE, LATENCY);

    /* WAVデータの再生 */
    while (fread(pcmBuffer, buffSize, 1, fp) == 1) {
        snd_pcm_writei(handle, (const void *)pcmBuffer, BUF_SAMPLES);
    }

    /* 後処理 */
    snd_pcm_drain(handle);
    snd_pcm_close(handle);
    fclose(fp);
    free(pcmBuffer);


    // 既存のplayWAVコードをここにコピーします
    // 省略
    // ...
    // 再生終了後、スレッドを終了する
    return NULL;
}

void startPlayWAV(const char *fileName) {
    pthread_t thread_id;
    // スレッドを作成してplayWAVを呼び出す
    pthread_create(&thread_id, NULL, playWAV, (void *)fileName);
    pthread_detach(thread_id); // スレッドをデタッチしてリソースを自動的に解放
}

#endif
