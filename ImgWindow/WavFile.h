/* WavFile.h
 *
 * Class(es) for reading audio samples
 *
 * Copyright (C) 2018, Christopher Collins
 */

/*
 * This module is only intended to support a narrow subset of possible Wav 
 * files, namely: PCM only.
*/

#ifndef _WAVFILE_H
#define _WAVFILE_H

#include <cstdlib>
#include <cstdint>

class AudioSampleData {
protected:
	int8_t		mNumChannels;
	int8_t		mBitsPerSample;
	uint8_t		mSampleAlignment;
	int 		mSampleRate;

	unsigned	mSampleCount;
	void *		mSampleData;

	bool		mIsFloat;

public:
	AudioSampleData(int numChannels, int bitsPerSample, int sampleRate, bool isFloat=false);
	AudioSampleData(AudioSampleData &&move_src);
	AudioSampleData(const AudioSampleData &cpy_src);
	virtual			~AudioSampleData();

	int8_t 			getNumChannels() const;
	int8_t			getBitsPerSample() const;
	uint8_t			getSampleAlignment() const;
	int				getSampleRate() const;
	size_t			getSampleCount() const;
	const void *	getSampleData() const;
	bool			isFloat() const;
	void			AppendSamples(uint8_t blockSize, unsigned count, void *data);
};

AudioSampleData *	LoadWav(const char *fileName);

#endif /* _WAVFILE_H */