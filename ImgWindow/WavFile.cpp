/* WavFile.cpp
 *
 * Class(es) for reading audio samples from WAV/RIFF files
 *
 * Copyright (C) 2018, Christopher Collins
 */

#include "WavFile.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

/*
 * There is a HUGE assumption in here right now that everything is 
 * Little Endian - this is now true for X-Plane.  It will need to be fixed
 * if this ever changes.
 *
 * - CC.
*/

/** minimumBlockAlignment calculates the minimum permissible block size for the
 * specified sampleSize & channel count.
 *
 * It does this obeying the rules established by Microsoft for multichannel
 * audio data & WAVE file, which in short, it is required that individual 
 * samples are padded up to a full byte, with channels in sequential order.
 *
 * There's nothing to say that a larger block alignment can't be used, which 
 * results in with wasted space.
 *
 * If a file reports a block alignment smaller than this, it must be invalid.
 *
 * We will repack the audio down to this size on load as not to waste memory.
 *
 * @param sampleSize size of each sample in bits.
 * @param numChannels total number of channels per sample
*/
inline int 
minimumBlockAlignment(int sampleSize, int numChannels)
{
	return ((sampleSize + 7) / 8) * numChannels;
}


AudioSampleData::AudioSampleData(int numChannels, int bitsPerSample, int sampleRate, bool isFloat) :
	mNumChannels(numChannels), 
	mBitsPerSample(bitsPerSample), 
	mSampleRate(sampleRate),
	mIsFloat(isFloat)
{
	mSampleAlignment = minimumBlockAlignment(mBitsPerSample, mNumChannels);
	mSampleCount = 0;
	mSampleData = nullptr;
}

AudioSampleData::AudioSampleData(AudioSampleData &&move_src) :
	mNumChannels(move_src.mNumChannels),
	mBitsPerSample(move_src.mBitsPerSample),
	mSampleAlignment(move_src.mSampleAlignment),
	mSampleRate(move_src.mBitsPerSample)
{
	mSampleData = move_src.mSampleData;
	mSampleCount = move_src.mSampleCount;
	move_src.mSampleData = nullptr;
	move_src.mSampleCount = 0;
}

AudioSampleData::AudioSampleData(const AudioSampleData &cpy_src) :
	mNumChannels(cpy_src.mNumChannels),
	mBitsPerSample(cpy_src.mBitsPerSample),
	mSampleAlignment(cpy_src.mSampleAlignment),
	mSampleRate(cpy_src.mBitsPerSample)
{
	mSampleCount = cpy_src.mSampleCount;
	mSampleData = malloc(cpy_src.mSampleCount * cpy_src.mSampleAlignment);
	if (mSampleData == nullptr) {
		mSampleCount = 0;
	} else {
		memcpy(mSampleData, cpy_src.mSampleData, cpy_src.mSampleCount * cpy_src.mSampleAlignment);
	}
}

AudioSampleData::~AudioSampleData()
{
	if (mSampleData != nullptr) {
		free(mSampleData);
		mSampleData = nullptr;
		mSampleCount = 0;
	}
}

void
AudioSampleData::AppendSamples(uint8_t blockSize, unsigned count, void *data)
{
	const unsigned newCount = mSampleCount + count;
	mSampleData = realloc(mSampleData, newCount*mSampleAlignment);
	if (mSampleData == nullptr) {
		mSampleCount = 0;
		return;
	}
	auto *dptr = reinterpret_cast<uint8_t *>(mSampleData);
	dptr += mSampleCount * mSampleAlignment;
	if (blockSize == mSampleAlignment) {
		memcpy(dptr, data, count*blockSize);		
	} else {
		auto *sptr = reinterpret_cast<uint8_t *>(data);
		for (unsigned c = 0; c < count; c++) {
			memcpy(dptr, sptr, mSampleAlignment);
			sptr += blockSize;
			dptr += mSampleAlignment;
		}
	}
	mSampleCount = newCount;
}

/* ==== Getters ==== */

int8_t
AudioSampleData::getNumChannels() const
{
	return mNumChannels;

}

int8_t
AudioSampleData::getBitsPerSample() const
{
	return mBitsPerSample;
}

uint8_t
AudioSampleData::getSampleAlignment() const
{
	return mSampleAlignment;
}

int
AudioSampleData::getSampleRate() const
{
	return mSampleRate;
}

size_t
AudioSampleData::getSampleCount() const
{
	return mSampleCount;
}

const void *
AudioSampleData::getSampleData() const
{
	return mSampleData;
}

bool
AudioSampleData::isFloat() const
{
	return mIsFloat;
}

/* ==== WAVE Loading Logic Below ==== */

static const char *	WavTopChunkID = "RIFF";
static const char *	WavFormatChunkID = "fmt ";
static const char *	WavDataChunkID = "data";

#pragma pack(push,1)
struct ChunkHeader {
	char		chunkID[4];
	uint32_t	chunkSize;	
};

/* names below from the Microsoft headers/documentation for consistency.
*/
struct WavFormatChunk {
	uint16_t	wFormatTag;
	uint16_t	nChannels;
	uint32_t	nSamplesPerSec;
	uint32_t	nAvgBytesPerSec;
	uint16_t	nBlockAlign;
	uint16_t	wBitsPerSample;
	/* 18 byte body... */
	uint16_t	cbSize;
	/* 40 byte body */
	uint16_t	wValidBitsPerSample;
	uint32_t	dwChannelMask;
	char 		SubFormat[16];
};
#pragma pack(pop)

const uint16_t	WAV_FORMAT_PCM = 1;
const uint16_t	WAV_FORMAT_IEEE_FLOAT = 3;

typedef struct WavTOCEntry {
	struct ChunkHeader	ch;
	uint32_t			offset;
} WavTOCEntry;

static vector<WavTOCEntry>::const_iterator
FindTOCFor(const vector<WavTOCEntry> &toc, const char *chunk_id)
{
	return std::find_if(toc.cbegin(), toc.cend(), [chunk_id](const WavTOCEntry &x) -> bool {return !memcmp(x.ch.chunkID, chunk_id, 4); });
}

static bool
readHeader(FILE *srcFile, const struct ChunkHeader &ch, vector<WavTOCEntry> &o_HeaderItems)
{
	uint32_t	expectedBytesLeft = ch.chunkSize - 4;

	while (expectedBytesLeft > 0) {
		/* if there's less than a chunkheader less, there's a problem. */
		if (expectedBytesLeft < sizeof(struct ChunkHeader)) {
			return false;
		}

		WavTOCEntry e;
		if (1 != fread(&e.ch, sizeof(e.ch), 1, srcFile)) {
			return false;
		}
		expectedBytesLeft -= sizeof(e.ch);

		/* make sure the expected file size is large enough */
		if (e.ch.chunkSize > expectedBytesLeft) {
			return false;
		}
		e.offset = ftell(srcFile);
		o_HeaderItems.emplace_back(e);

		int pad = 0;
		if ((e.ch.chunkSize % 2) != 0) {
			pad = 1;
		}

		/* skip forward to the next chunk */
		fseek(srcFile, e.ch.chunkSize + pad, SEEK_CUR);
		expectedBytesLeft -= e.ch.chunkSize + pad;
	}
	return true;
}

static 
AudioSampleData *extractData(FILE* fh, const vector<WavTOCEntry> &toc)
{
	uint8_t *dbuf = nullptr;
	AudioSampleData *asd = nullptr;
	size_t samplesRead = 0;

	/* if we get here without an error, it means the file was well formed and
	 * we now have a ToC for the file chunks 
	 * 
	 * Find the format header.
	 */
	auto ti = FindTOCFor(toc, WavFormatChunkID);
	if (ti == toc.end()) {
		// couldn't find the format chunk
		return nullptr;
	}
	fseek(fh, ti->offset, SEEK_SET);
	int chunkSize = ti->ch.chunkSize;

	/* safety check - the chunk can't be smaller than 16 bytes */
	if (chunkSize < 16) {
		return nullptr;
	}
	/* and cap it at the size of the header struct so we don't buffer overflow */
	if (chunkSize > sizeof(WavFormatChunk)) {
		chunkSize = sizeof(WavFormatChunk);
	}

	struct WavFormatChunk	fc;
	if (1 != fread(&fc, chunkSize, 1, fh)) {
		return nullptr;
	}

	/* now that we have the header, verify that we can support the format. */
	if (fc.wFormatTag != WAV_FORMAT_PCM && fc.wFormatTag != WAV_FORMAT_IEEE_FLOAT) {
		return nullptr;
	}
	/* sanity check the block stride */
	if (fc.nBlockAlign < minimumBlockAlignment(fc.wBitsPerSample, fc.nChannels)) {
		return nullptr;
	}

	asd = new AudioSampleData(fc.nChannels, fc.wBitsPerSample, fc.nSamplesPerSec, fc.wFormatTag == WAV_FORMAT_IEEE_FLOAT);

	/* now to check the data block */
	ti = FindTOCFor(toc, WavDataChunkID);
	if (ti == toc.end()) {
		goto fail2;
	}

	/* we can live with short reads here, so we use a slightly different strategy */
	fseek(fh, ti->offset, SEEK_SET);

	dbuf = reinterpret_cast<uint8_t *>(malloc(ti->ch.chunkSize));
	if (nullptr == dbuf) {
		goto fail2;
	}
	samplesRead = fread(dbuf, fc.nBlockAlign, ti->ch.chunkSize / fc.nBlockAlign, fh);
	if (samplesRead <= 0) {
		goto fail;
	}
	asd->AppendSamples(fc.nBlockAlign, static_cast<unsigned>(samplesRead), dbuf);	
	return asd;
fail:
	free(dbuf);
fail2:
	delete asd;
	return nullptr;
}

AudioSampleData *
LoadWav(const char *fileName)
{
	FILE *fh = nullptr;
	vector<WavTOCEntry>	toc;
	AudioSampleData* asd = nullptr;

	fh = fopen(fileName, "rb");
	if (fh == nullptr) {
		return nullptr;
	}

	/* first of all, verify that we have an actual wavfile */
	struct ChunkHeader ch;

	if (1 != fread(&ch, sizeof(ch), 1, fh)) {
		/* short read or error */
		goto fail;
	}
	/* check the top level header. */
	if (memcmp(WavTopChunkID, ch.chunkID, 4)) {
		goto fail;
	}
	/* minimum sensible size is the WAVE identifier, 2 chunk headers 
	 * and the minimum legal WAVFormatChunk body 
	 */
	if (ch.chunkSize < (4+16+8+8)) {
		goto fail;
	}
	/* check the WAVE magic */
	char waveMagic[4];
	if (1 != fread(&waveMagic, 4, 1, fh)) {
		goto fail;
	}
	if (memcmp("WAVE", waveMagic, 4)) {
		goto fail;
	}

	/* if were're still here, the file is valid so far, now we need to read
	 * scan the wave file to find the blocks we need */
	if (!readHeader(fh, ch, toc)) {
		goto fail;
	}

	asd = extractData(fh, toc);
	if (asd == nullptr) {
		goto fail;
	}

	return asd;
fail:
	if (fh != nullptr) {
		fclose(fh);
		fh = nullptr;
	}
	return nullptr;
}
