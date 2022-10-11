#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

#ifndef PI
#define PI 3.14159265359
#endif

class Oscillator
{
    double frequency;
    int sampleIndex;
    int samplesPerCycle;
    double sampleRate;

public:
    
    Oscillator ()
    {
        sampleIndex = 0;
        frequency = 441.0;
        sampleRate = 48000.0;
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
        if (sampleIndex >= samplesPerCycle) sampleIndex = 0;
        
        return sample;
    }
};


void writeToFile (std::ofstream &file, const int value, const int size) {
    file.write (reinterpret_cast<const char*> (&value), size);
}

void writeVectorToFile (const std::vector<float> bufferData, const double sampleRate)
{
   const int bitDepth = 24;
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

   auto maxAmplitude = pow (2, bitDepth - 1) - 1;
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
    double sampleRate = 48000;
    Oscillator sinGenerator;

    sinGenerator.setFrequency (frequency);
    sinGenerator.setSampleRate (sampleRate);
    
    std::vector<float> bufferData;
    int numSamplesInLength = lengthInSeconds * sampleRate;
    
    while (numSamplesInLength--)
    {
        bufferData.push_back (sinGenerator.getNextSample());
    }

    writeVectorToFile (bufferData, sampleRate);
}

int main()
{
    writeSinWaveToFile (440, 1.0);
    return 0;
}

/*
// Read in the file
//
wav.read((char*)(&header), sizeof(WaveHeader));
numSamples = header.dataChunkSize / 2;
cout << numSamples << endl;
// Create a dynamic array of samples
waveData = new int16_t[numSamples];
wav.read((char*)(waveData), header.dataChunkSize);
*/
