# CompanyCodingTestAnswers

Code written for company interview tests, kept because the problems were worth
keeping.

`WriteAudioToWaveFile.cpp` writes a buffer of samples out as a RIFF WAVE file
with the header assembled by hand: the RIFF and WAVE chunk ids, the `fmt ` chunk
carrying channel count, sample rate, byte rate, block align and bit depth, and
the `data` chunk with the sizes filled in after the samples are written.

Writing a WAV by hand is a good interview question because the format is small
enough to hold in your head and unforgiving about the two size fields, which are
the part most answers get wrong.
