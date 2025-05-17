/***************************************************
 * LZ77 Compression Implementation for LoRa Mesh
 * 
 * Optimized for small memory footprint on ESP32
 * Uses a 512-byte sliding window as specified
 ***************************************************/

#ifndef LZ77_H
#define LZ77_H

#include <Arduino.h>

class LZ77 {
private:
  // Sliding window parameters
  static const uint16_t WINDOW_SIZE = 512;
  static const uint8_t MIN_MATCH_LENGTH = 3;
  static const uint16_t MAX_MATCH_LENGTH = 255;
  
  uint8_t window[WINDOW_SIZE];
  uint16_t windowPosition;
  
  // Find the longest match in the sliding window
  uint16_t findLongestMatch(const uint8_t* data, uint16_t dataPos, uint16_t dataSize, uint16_t& matchLength) {
    uint16_t bestMatchPos = 0;
    matchLength = 0;
    
    // Don't attempt to match if we have too little data
    if (dataSize - dataPos < MIN_MATCH_LENGTH) {
      return 0;
    }
    
    for (uint16_t i = 0; i < windowPosition; i++) {
      // Calculate how many bytes match
      uint16_t matchLen = 0;
      while ((dataPos + matchLen < dataSize) && 
             (matchLen < MAX_MATCH_LENGTH) && 
             (data[dataPos + matchLen] == window[(i + matchLen) % WINDOW_SIZE])) {
        matchLen++;
      }
      
      // Update if this is the best match so far
      if (matchLen > matchLength) {
        matchLength = matchLen;
        bestMatchPos = i;
        
        // If we found a match of maximum length, stop searching
        if (matchLength == MAX_MATCH_LENGTH) {
          break;
        }
      }
    }
    
    // Return 0 if the match is too short
    if (matchLength < MIN_MATCH_LENGTH) {
      matchLength = 0;
      return 0;
    }
    
    return bestMatchPos;
  }
  
public:
  LZ77() : windowPosition(0) {
    memset(window, 0, WINDOW_SIZE);
  }
  
  uint16_t compress(const uint8_t* input, uint16_t inputSize, uint8_t* output, uint16_t outputSize) {
    if (!input || !output || inputSize == 0 || outputSize == 0) {
      return 0;
    }
    
    // Clear the sliding window
    memset(window, 0, WINDOW_SIZE);
    windowPosition = 0;
    
    uint16_t inputPos = 0;
    uint16_t outputPos = 0;
    
    // Reserve first byte for flags
    uint8_t flagsPos = outputPos++;
    uint8_t flags = 0;
    uint8_t flagsMask = 1;
    
    while (inputPos < inputSize && outputPos < outputSize - 2) { // -2 for safety
      // Check if we need a new flags byte
      if (flagsMask == 0) {
        // Store the flags byte
        output[flagsPos] = flags;
        
        // Start a new flags byte
        flagsPos = outputPos++;
        flags = 0;
        flagsMask = 1;
        
        // Ensure we have space for at least one more token
        if (outputPos >= outputSize - 2) {
          break;
        }
      }
      
      // Find the longest match in the window
      uint16_t matchLength;
      uint16_t matchPos = findLongestMatch(input, inputPos, inputSize, matchLength);
      
      if (matchLength >= MIN_MATCH_LENGTH) {
        // We found a match - encode a reference
        flags |= flagsMask; // Set the flag bit
        
        // Encode match position and length
        output[outputPos++] = matchPos & 0xFF;
        output[outputPos++] = ((matchPos >> 8) & 0x0F) | ((matchLength - MIN_MATCH_LENGTH) << 4);
        
        // Add the matched bytes to the window
        for (uint16_t i = 0; i < matchLength; i++) {
          window[windowPosition] = input[inputPos];
          windowPosition = (windowPosition + 1) % WINDOW_SIZE;
          inputPos++;
        }
      } else {
        // No match - encode a literal byte
        // The flag bit is already 0
        output[outputPos++] = input[inputPos];
        
        // Add the byte to the window
        window[windowPosition] = input[inputPos];
        windowPosition = (windowPosition + 1) % WINDOW_SIZE;
        inputPos++;
      }
      
      // Move to the next flag bit
      flagsMask <<= 1;
    }
    
    // Store the final flags byte
    output[flagsPos] = flags;
    
    // Return the size of the compressed data
    return (inputPos == inputSize) ? outputPos : 0;
  }
  
  uint16_t decompress(const uint8_t* input, uint16_t inputSize, uint8_t* output, uint16_t outputSize) {
    if (!input || !output || inputSize == 0 || outputSize == 0) {
      return 0;
    }
    
    // Clear the sliding window
    memset(window, 0, WINDOW_SIZE);
    windowPosition = 0;
    
    uint16_t inputPos = 0;
    uint16_t outputPos = 0;
    
    while (inputPos < inputSize && outputPos < outputSize) {
      // Read the flags byte
      uint8_t flags = input[inputPos++];
      uint8_t flagsMask = 1;
      
      // Process 8 bits of flags (or until end of input/output)
      while (flagsMask && inputPos < inputSize && outputPos < outputSize) {
        if (flags & flagsMask) {
          // This is a match
          if (inputPos + 1 >= inputSize) break;
          
          uint16_t matchPos = input[inputPos++];
          uint8_t  matchInfo = input[inputPos++];
          
          matchPos |= ((matchInfo & 0x0F) << 8);
          uint16_t matchLength = ((matchInfo >> 4) & 0x0F) + MIN_MATCH_LENGTH;
          
          // Copy the matched bytes to the output and window
          for (uint16_t i = 0; i < matchLength && outputPos < outputSize; i++) {
            uint8_t byte = window[matchPos % WINDOW_SIZE];
            output[outputPos++] = byte;
            
            window[windowPosition] = byte;
            windowPosition = (windowPosition + 1) % WINDOW_SIZE;
            
            matchPos++;
          }
        } else {
          // This is a literal byte
          if (inputPos >= inputSize) break;
          
          uint8_t byte = input[inputPos++];
          output[outputPos++] = byte;
          
          // Add the byte to the window
          window[windowPosition] = byte;
          windowPosition = (windowPosition + 1) % WINDOW_SIZE;
        }
        
        // Move to the next flag bit
        flagsMask <<= 1;
      }
    }
    
    return outputPos;
  }
  
  // Delta encoding for sensor data
  uint16_t deltaEncode(const uint8_t* input, uint16_t inputSize, uint8_t* output, uint16_t outputSize) {
    if (inputSize < 2 || outputSize < inputSize) {
      return 0;
    }
    
    output[0] = input[0]; // Store the first value directly
    uint16_t outputPos = 1;
    
    for (uint16_t i = 1; i < inputSize && outputPos < outputSize; i++) {
      // Calculate and store delta (difference from previous value)
      int8_t delta = input[i] - input[i-1];
      output[outputPos++] = delta;
    }
    
    return outputPos;
  }
  
  uint16_t deltaDecode(const uint8_t* input, uint16_t inputSize, uint8_t* output, uint16_t outputSize) {
    if (inputSize < 1 || outputSize < inputSize) {
      return 0;
    }
    
    output[0] = input[0]; // First value is stored directly
    
    for (uint16_t i = 1; i < inputSize && i < outputSize; i++) {
      // Reconstruct value by adding delta to previous value
      output[i] = output[i-1] + (int8_t)input[i];
    }
    
    return inputSize;
  }
};

#endif // LZ77_H
