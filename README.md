A little demo projects merging two uint16_t buffers into one uint32_t over dma triggered by timer.
Example 

Source buffers:

        uint16_t src1[] = { 0x00AA, 0x001B ... }
        uint16_t src2[] = { 0x00DE, 0x002C ... }

The destination buffer will be:

	uint32_t dst[] = { 0x00DE00AA, 0x002C001B ... }

The DMA transferrs are triggered by timer, where 2 DMA  streams with high priority
write two uint16_t halfs into one merged uint32_t value, and the third, low priority
DMA stream, copies the combined value into output uint32_t buffer. I needed it for 
merging two mono audio streams, arriving from 2 microphones, into one stereo stream.
