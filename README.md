# CompanyCodingTestAnswers

Code submitted as part of company interview tests.

## What this is

Code written for company interview tests, kept because the problems were worth keeping. There is one program, `WriteAudioToWaveFile.cpp`. It generates a pure sine wave (the simplest musical tone, a smooth wave at one frequency) and saves it as a WAV file, the standard uncompressed audio format. The test asked for a 24-bit, mono, 48 kHz file. When you run it, it writes one second of a 7040 Hz tone to `waveform.wav`, which you can open in any audio player.

## What you can learn from it

- Music Technology: what sample rate and bit depth mean, and how a tone becomes a list of numbers.
- Physics and Maths: generating a sine wave from its frequency, and the Nyquist limit (the highest frequency a sample rate can hold, half the sample rate).
- Computer Science: binary file formats, little-endian byte order, and writing a header whose size fields are filled in after the data.
- Computer Science: checking input and reporting errors with exceptions.
- Signal processing: estimating pitch by counting zero crossings (the points where the wave crosses zero).

## Run it

### On your own computer

**You will need:** a C++ compiler. On macOS, install the Xcode Command Line Tools (`xcode-select --install`). On Linux, install `g++`. On Windows, use [Visual Studio](https://visualstudio.microsoft.com/) with the C++ workload. Tested on macOS with Apple clang 17.

1. Download the code:

   ```sh
   git clone https://github.com/godofecht/CompanyCodingTestAnswers.git
   cd CompanyCodingTestAnswers
   ```

2. Compile it:

   ```sh
   c++ -std=c++17 -o wav WriteAudioToWaveFile.cpp
   ```

3. Run it:

   ```sh
   ./wav
   ```

The program prints nothing. It creates `waveform.wav` (144,044 bytes) in the current folder. Play it and you hear one second of a high, steady tone.

## How it works

A WAV file starts with a short header that describes the sound, then holds the samples. Each sample is one number that says where the wave is at that instant. At 48,000 samples a second and 3 bytes (24 bits) per sample, one second of mono sound takes 144,000 bytes.

The program writes the header first with placeholder sizes (`----`), then writes every sample, then goes back and fills in the two size fields once it knows how much data there is.

| Part of the file | What is in it |
|---|---|
| `writeSinWaveToFileNonVerbose` | Checks the frequency, writes the header, computes each sample and writes it straight to the file. `main` calls this one. |
| `writeSinWaveToFile` | The same job in two steps: fills a `std::vector` using the `Oscillator` class, checks the pitch, then calls `writeVectorToFile`. |
| `Oscillator` | Produces one sine sample at a time and wraps its position back after each full cycle. |
| `getNumZeroCrossings`, `getPitchFromNumZeroCrossings` | Count zero crossings in the buffer and turn the count into a frequency, as a check. |
| The commented-out block at the top | An earlier version that writes `audio.wav` from a packed header `struct`. It is not compiled. |

## Ideas to try

1. Change the tone. In `WriteAudioToWaveFile.cpp`, edit `writeSinWaveToFileNonVerbose (7040, 1.0)` in `main` to 440 Hz (the A above middle C) for three seconds.
2. Try a frequency above 24,000 Hz. The program prints `Frequency invalid, above Nyquist` and exits. Then read why the Nyquist limit exists.
3. Switch `main` to call `writeSinWaveToFile` instead. It prints the requested pitch and the pitch measured by counting zero crossings. For 440 Hz it prints 439.5. Work out where the half a hertz goes.
4. Change `bitDepth` from 24 to 16 in `writeSinWaveToFileNonVerbose` and compare the file sizes.
5. Write a stereo file. You will need to change the channel count, the byte rate and the block align in the header, and write two samples per frame.

## Technical details

`WriteAudioToWaveFile.cpp` writes a buffer of samples out as a RIFF WAVE file with the header assembled by hand: the RIFF and WAVE chunk ids, the `fmt ` chunk carrying channel count, sample rate, byte rate, block align and bit depth, and the `data` chunk with the sizes filled in after the samples are written.

Writing a WAV by hand is a good interview question because the format is small enough to hold in your head and unforgiving about the two size fields, which are the part most answers get wrong.

Here the `data` chunk size is written at `preAudioPosition - 4`, and the RIFF chunk size (file length minus 8) at byte 4, using `seekp` once the samples are written. Samples are scaled by 2^23 - 1 and written as the low 3 bytes of an `int`, which gives 24-bit little-endian signed PCM on a little-endian machine. The format code is 1 (PCM).

## Project status

Finished interview answer. The output was checked on macOS: `afinfo` and Python's `wave` module both read `waveform.wav` as 1 channel, 48,000 Hz, 24-bit, 48,000 frames.

The code uses `std::invalid_argument` without including `<stdexcept>`, so some compilers may need that header added. The test asked for `audio.wav`; the program writes `waveform.wav`.

## Licence

You can use, copy and change this code for any purpose, as long as you keep the copyright notice. See [LICENSE](LICENSE) (MIT).

## Credits

Written by Abhishek Shivakumar.
