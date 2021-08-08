#pragma once

#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
typedef struct {
	float* analogIn;
	float* analogOut;
	const size_t analogInChannels;
	const size_t analogOutChannels;
	const size_t analogFrames;
	float analogSampleRate;
	uint32_t* digital;
	const size_t digitalChannels;
	const uint32_t digitalFrames;
	const float digitalSampleRate;
} BelaContext;

void render(BelaContext *context, void *userData);

// analogRead()
//
// Returns the value of the given analog input at the given frame number.
static inline float analogRead(BelaContext *context, int frame, int channel) {
	return context->analogIn[frame * context->analogInChannels + channel];
}

static inline float analogReadNI(BelaContext *context, int frame, int channel) {
	return context->analogIn[channel * context->analogFrames + frame];
}

// analogWriteOnce()
//
// Sets a given channel to a value for only the current frame
static inline void analogWriteOnce(BelaContext *context, int frame, int channel, float value) {
	context->analogOut[frame * context->analogOutChannels + channel] = value;
}

static inline void analogWriteOnceNI(BelaContext *context, int frame, int channel, float value) {
	context->analogOut[channel * context->analogFrames + frame] = value;
}

// analogWrite()
//
// Sets a given analog output channel to a value for the current frame and, if persistent outputs are
// enabled, for all subsequent frames
static inline void analogWrite(BelaContext *context, int frame, int channel, float value) {
	unsigned int f;
	for(f = frame; f < context->analogFrames; f++)
		analogWriteOnce(context, f, channel, value);
}

static inline void analogWriteNI(BelaContext *context, int frame, int channel, float value) {
	unsigned int f;
	for(f = frame; f < context->analogFrames; f++)
		analogWriteOnceNI(context, f, channel, value);
}

enum {
	INPUT = 0,
	OUTPUT = 1,
};
/// Set the given bit in \c word to 1.
#define Bela_setBit(word,bit) 			((word) | (1 << (bit)))

/// Clear the given bit in \c word to 0.
#define Bela_clearBit(word,bit) 			((word) &~ (1 << (bit)))

/// Check if the given bit in \c word is 1 (returns nonzero) or 0 (returns zero).
#define Bela_getBit(word,bit) 			(((word) >> (bit)) & 1)

/// Set/clear the given bit in \c word to \c value.
#define Bela_changeBit(word,bit,value) 	((Bela_clearBit((word),(bit))) | ((value) << (bit)))
// digitalRead()
//
// Returns the value of a given digital input at the given frame number
static inline int digitalRead(BelaContext *context, int frame, int channel) {
	return Bela_getBit(context->digital[frame], channel + 16);
}

// digitalWrite()
//
// Sets a given digital output channel to a value for the current frame and all subsequent frames
static inline void digitalWrite(BelaContext *context, int frame, int channel, int value) {
	unsigned int f;
	for(f = frame; f < context->digitalFrames; f++) {
		if(value)
			context->digital[f] |= 1 << (channel + 16);
		else
			context->digital[f] &= ~(1 << (channel + 16));
	}
}

// digitalWriteOnce()
//
// Sets a given digital output channel to a value for the current frame only
static inline void digitalWriteOnce(BelaContext *context, int frame, int channel, int value) {
	if(value)
		context->digital[frame] |= 1 << (channel + 16);
	else
		context->digital[frame] &= ~(1 << (channel + 16));
}

// pinMode()
//
// Sets the direction of a digital pin for the current frame and all subsequent frames
static inline void pinMode(BelaContext *context, int frame, int channel, int mode) {
	unsigned int f;
	for(f = frame; f < context->digitalFrames; f++) {
		if(mode == INPUT)
			context->digital[f] |= (1 << channel);
		else
			context->digital[f] &= ~(1 << channel);
	}
}

// pinModeOnce()
//
// Sets the direction of a digital pin for the current frame only
static inline void pinModeOnce(BelaContext *context, int frame, int channel, int mode) {
	if(mode == INPUT)
		context->digital[frame] |= (1 << channel);
	else
		context->digital[frame] &= ~(1 << channel);
}
#ifdef __cplusplus
}
#endif // __cplusplus
