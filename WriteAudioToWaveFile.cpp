#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

#ifndef PI
#define PI 3.14159265359
#endif



//Yuriy from AudioMovers
/*

Hi Abhishek, so as discussed the task

Write a function in c++ which creates an audio wave file with pure sinusoidal sound.
on input: sound frequency in Hertzs and a length of the file in seconds
on output:
1. audio.wav file 24 bit, mono, 48Khz in current directory. OR
2. an exception with an explanation if there was an error anywhere.

Thanks,
Yuriy

*/


/*
I've never worked directly with WAV files before. I did the research and wrote a function in about 15 minutes and sent it over.
Got some feedback. Fixed all the points in 15 more minutes. Lost the job offer on the basis of the initial submission.
I've just closed a deal with a client and have some free time.
So here goes. You want verbosity, you can have it.
*/


float getPitchFromNumZeroCrossings (const int numZeroCrossings, const int numSamples, const double sampleRate)
{
    const float bufferTimeInSeconds = (float) numSamples / (float) sampleRate;
    const float numCycles = (float) numZeroCrossings / 2.0f;
    const float pitchInHertz = numCycles / bufferTimeInSeconds;
    
    return pitchInHertz;
}

int getNumZeroCrossings (const std::vector<double>& buffer)
{
    int numZeroCrossings = 0;

    for (int sample = 1; sample < buffer.size(); ++sample)
    {
        if (((buffer[sample - 1] > 0.0f) && (buffer[sample] <= 0.0f))
         || ((buffer[sample - 1] < 0.0f) && (buffer[sample] >= 0.0f)))
        {
            ++numZeroCrossings;
        }
    }

    return numZeroCrossings;
}

class Oscillator
{
    double frequency;
    double sampleIndex;
    double samplesPerCycle;
    double sampleRate;

public:
    
    Oscillator (double newFrequency, double newSampleRate)
    {
        sampleIndex = 0;
        frequency = newFrequency;
        sampleRate = newSampleRate;
        samplesPerCycle = sampleRate / frequency;
    }
    
    void setSampleRate (const double newSampleRate)
    {
        sampleRate = newSampleRate;
        samplesPerCycle = sampleRate / frequency;
    }
    
    void setFrequency (const double newFrequency)
    {
        frequency = newFrequency;
        samplesPerCycle = sampleRate / frequency;
    }
    
    double getNextSample()
    {
        double sample = (1.0 * sin (2.0 * PI * (double) sampleIndex / (double) samplesPerCycle));
        sampleIndex++;
        
        //reset the sampleIndex after each cycle
        if (sampleIndex >= samplesPerCycle)
            sampleIndex = sampleIndex - samplesPerCycle;
        
        return sample;
    }
};


void writeToFile (std::ofstream &file, const int value, const int size)
{
    file.write (reinterpret_cast<const char*> (&value), size);
}


void writeSinWaveToFileNonVerbose (const double frequency, const double lengthInSeconds)
{
    double sample, intSample;
    int bitDepth = 24;
    float sampleIndex = 0;
    float sampleRate = 48000;
    float samplesPerCycle = sampleRate / frequency;
    float numSamplesInLength = lengthInSeconds * sampleRate;
    //Check range
    if (frequency < 0)
    {
        throw std::invalid_argument ("Frequency invalid, below 0");
        return;
    }
    else if (frequency > sampleRate / 2.0)
    {
        throw std::invalid_argument ("Frequency invalid, above Nyquist");
        return;
    }
    
    std::ofstream file;
    file.open ("waveform.wav", std::ios::binary);
     
    //Header chunk
    file << "RIFF";
    file << "----";
    file << "WAVE";

    //Format chunk
    file << "fmt ";
    writeToFile (file, 16, 4); // Size
    writeToFile (file, 1, 2); // Compression code
    writeToFile (file, 1, 2); // Number of channels
    writeToFile (file, sampleRate, 4); // Sample rate
    writeToFile (file, sampleRate * bitDepth / 8, 4 ); // Byte rate
    writeToFile (file, bitDepth / 8, 2); // Block align
    writeToFile (file, bitDepth, 2); // Bit depth

    //Data chunk
    file << "data";
    file << "----";

    int preAudioPosition = file.tellp();
    auto maxAmplitude = powf (2, bitDepth - 1) - 1;
    
    for (int i = 0; i < numSamplesInLength; i++ )
    {
        //reset the sampleIndex after each cycle
        if (sampleIndex >= samplesPerCycle)
            sampleIndex = sampleIndex - samplesPerCycle;
        
        sample = (1.0 * sin (2.0 * PI * sampleIndex / samplesPerCycle));

        intSample = sample * maxAmplitude;
        writeToFile (file, intSample, (bitDepth / 8));
        
        sampleIndex++;
    }
    
    int postAudioPosition = file.tellp();
    file.seekp (preAudioPosition - 4);
    writeToFile (file, postAudioPosition - preAudioPosition, 4);
    file.seekp (4, std::ios::beg);
    writeToFile (file, postAudioPosition - 8, 4);
    file.close();
}

void writeVectorToFile (const std::vector<double> bufferData, const double sampleRate)
{
   int bitDepth = 24;
   std::ofstream file;
   file.open ("waveform.wav", std::ios::binary);
    
   //Header chunk
   file << "RIFF";
   file << "----";
   file << "WAVE";

   //Format chunk
   file << "fmt ";
   writeToFile (file, 16, 4); // Size
   writeToFile (file, 1, 2); // Compression code
   writeToFile (file, 1, 2); // Number of channels
   writeToFile (file, sampleRate, 4); // Sample rate
   writeToFile (file, sampleRate * bitDepth / 8, 4 ); // Byte rate
   writeToFile (file, bitDepth / 8, 2); // Block align
   writeToFile (file, bitDepth, 2); // Bit depth

   //Data chunk
   file << "data";
   file << "----";

   int preAudioPosition = file.tellp();

   auto maxAmplitude = powf (2, bitDepth - 1) - 1;
   for(int i = 0; i < bufferData.size(); i++ )
   {
        auto sample =  bufferData[i];
        double intSample = static_cast<double> (sample * maxAmplitude);
        writeToFile (file, intSample, (bitDepth / 8));
   }
   int postAudioPosition = file.tellp();

   file.seekp (preAudioPosition - 4);
   writeToFile (file, postAudioPosition - preAudioPosition, 4);

   file.seekp (4, std::ios::beg);
   writeToFile (file, postAudioPosition - 8, 4);

   file.close();
}


//This function will load a sin wave into an std::vector, and write the contents of the vector
//to a .wav file at 48kHz and 24 bit depth at the specified frequency, with the specified
//lengthInSeconds.
void writeSinWaveToFile (const double frequency, const double lengthInSeconds)
{
    double sampleRate = 48000.0;
    
    //Check range
    if (frequency < 0)
    {
        throw std::invalid_argument ("Frequency invalid, below 0");
        return;
    }
    else if (frequency > sampleRate / 2.0)
    {
        throw std::invalid_argument ("Frequency invalid, above Nyquist");
        return;
    }
    
    Oscillator sinGenerator (frequency, sampleRate);

    sinGenerator.setFrequency (frequency);
    sinGenerator.setSampleRate (sampleRate);
    
    std::vector<double> bufferData;
    int numSamplesInLength = lengthInSeconds * sampleRate;
    
    while (numSamplesInLength--)
    {
        bufferData.push_back (sinGenerator.getNextSample());
    }

    auto numCrossings = getNumZeroCrossings (bufferData);
    std::cout << "Expected pitch is: " << frequency << std::endl;
    std::cout << "Generated Pitch is: " << getPitchFromNumZeroCrossings (numCrossings, bufferData.size(), sampleRate);
    
    writeVectorToFile (bufferData, sampleRate);
}

int main()
{
    try
    {
        writeSinWaveToFileNonVerbose (7040, 1.0);
    }
    catch (std::invalid_argument& e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}




