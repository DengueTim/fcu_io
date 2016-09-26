#ifndef BLACKBOX_INPUT_STREAM_H
#define BLACKBOX_INPUT_STREAM_H

#include <stddef.h>
#include <stdbool.h>

namespace blackbox {

class ParserInputStream {
public:
	ParserInputStream(int (*getNextByte)());
	~ParserInputStream();

	int streamPeekChar();
	int streamReadChar();
	int streamReadByte();
//	void streamUnreadChar(, int c);

	void streamRead(void *buf, int len);

	uint32_t streamReadBits(int numBits);
	int streamReadBit();
	void streamByteAlign();

	uint32_t streamReadUnsignedVB();
	int32_t streamReadSignedVB();

private:
	int (*getNextByte_)();
	const char data_;

	//When reading bit-by-bit, the index of the next bit to be read within the byte at pos (from the high bit of index 7..0)
	int bitPos_;

	//Set to true if we attempt to read from the log when it is already exhausted
	bool eof_;
};

}


#endif
