/***************************************************
 * Reed-Solomon Forward Error Correction
 * 
 * Simplified implementation for ESP32 with limited RAM
 * Provides error detection and correction capability
 ***************************************************/

#ifndef RS_FEC_H
#define RS_FEC_H

#include <Arduino.h>

class RS_FEC {
private:
  // Galois field parameters
  static const uint8_t GF_SIZE = 16; // Use GF(16) for simplicity
  static const uint8_t GF_POLY = 0x13; // x^4 + x + 1 (primitive polynomial)
  
  // RS parameters
  static const uint8_t RS_N = 15; // Codeword length (15 for GF(16))
  static const uint8_t RS_K = 11; // Data length
  static const uint8_t RS_T = 2;  // Error correction capability ((N-K)/2)
  
  uint8_t gfLog[GF_SIZE];     // Log table
  uint8_t gfExp[GF_SIZE];     // Exponential table
  uint8_t gfPoly[RS_N-RS_K+1]; // Generator polynomial
  
  // Initialize Galois field tables
  void initGaloisField() {
    uint8_t x = 1;
    for (uint8_t i = 0; i < GF_SIZE-1; i++) {
      gfExp[i] = x;
      // Multiply by x in GF(16)
      x <<= 1;
      if (x & GF_SIZE) { // If x^4 is set
        x ^= GF_POLY;
      }
      x &= (GF_SIZE-1); // Keep in GF(16)
    }
    gfExp[GF_SIZE-1] = 1; // Ensure proper wrap-around
    
    // Build log table
    for (uint8_t i = 0; i < GF_SIZE-1; i++) {
      gfLog[gfExp[i]] = i;
    }
    gfLog[0] = 0; // Log of 0 is undefined, but set to 0 for simplicity
  }
  
  // Initialize generator polynomial
  void initGeneratorPoly() {
    gfPoly[0] = 1;
    
    // Compute (x + a^j) for j = 1 to 2*t
    for (uint8_t i = 0; i < 2*RS_T; i++) {
      gfPoly[i+1] = 1;
      
      for (int j = i; j > 0; j--) {
        if (gfPoly[j] == 0) {
          gfPoly[j] = gfPoly[j-1];
        } else {
          gfPoly[j] = gfPoly[j-1] ^ gfExp[(gfLog[gfPoly[j]] + i + 1) % (GF_SIZE-1)];
        }
      }
      
      gfPoly[0] = gfExp[(gfLog[gfPoly[0]] + i + 1) % (GF_SIZE-1)];
    }
  }
  
  // Multiply two polynomials in GF(16)
  void polyMult(uint8_t* a, uint8_t aLen, uint8_t* b, uint8_t bLen, uint8_t* result) {
    memset(result, 0, aLen + bLen - 1);
    
    for (uint8_t i = 0; i < aLen; i++) {
      for (uint8_t j = 0; j < bLen; j++) {
        if (a[i] != 0 && b[j] != 0) {
          result[i+j] ^= gfExp[(gfLog[a[i]] + gfLog[b[j]]) % (GF_SIZE-1)];
        }
      }
    }
  }
  
public:
  RS_FEC() {
    initGaloisField();
    initGeneratorPoly();
  }
  
  // Encode a message using Reed-Solomon
  uint8_t encode(const uint8_t* data, uint8_t dataLen, uint8_t* encoded) {
    if (dataLen > RS_K) {
      return 0; // Data too long
    }
    
    // Copy data to encoded buffer
    memcpy(encoded, data, dataLen);
    
    // Pad with zeros if needed
    for (uint8_t i = dataLen; i < RS_K; i++) {
      encoded[i] = 0;
    }
    
    // Calculate parity bytes
    uint8_t parity[RS_N-RS_K] = {0};
    
    for (uint8_t i = 0; i < RS_K; i++) {
      uint8_t feedback = encoded[i] ^ parity[0];
      
      if (feedback != 0) {
        uint8_t feedbackLog = gfLog[feedback];
        
        // Update parity bytes using generator polynomial
        for (uint8_t j = 0; j < RS_N-RS_K-1; j++) {
          if (gfPoly[j+1] == 0) {
            parity[j] = parity[j+1];
          } else {
            parity[j] = parity[j+1] ^ gfExp[(feedbackLog + gfLog[gfPoly[j+1]]) % (GF_SIZE-1)];
          }
        }
        
        parity[RS_N-RS_K-1] = gfPoly[RS_N-RS_K] == 0 ? 0 : gfExp[(feedbackLog + gfLog[gfPoly[RS_N-RS_K]]) % (GF_SIZE-1)];
      } else {
        // Shift parity bytes
        for (uint8_t j = 0; j < RS_N-RS_K-1; j++) {
          parity[j] = parity[j+1];
        }
        parity[RS_N-RS_K-1] = 0;
      }
    }
    
    // Append parity bytes to the encoded message
    for (uint8_t i = 0; i < RS_N-RS_K; i++) {
      encoded[RS_K+i] = parity[i];
    }
    
    return RS_N;
  }
  
  // Decode a Reed-Solomon encoded message
  // Returns number of corrected errors, -1 if uncorrectable
  int8_t decode(uint8_t* data, uint8_t dataLen) {
    if (dataLen != RS_N) {
      return -1; // Invalid length
    }
    
    // Calculate syndromes
    uint8_t syndromes[2*RS_T] = {0};
    bool hasErrors = false;
    
    for (uint8_t i = 0; i < 2*RS_T; i++) {
      uint8_t power = i + 1;
      for (uint8_t j = 0; j < RS_N; j++) {
        if (data[j] != 0) {
          syndromes[i] ^= gfExp[(gfLog[data[j]] + power * j) % (GF_SIZE-1)];
        }
      }
      if (syndromes[i] != 0) {
        hasErrors = true;
      }
    }
    
    if (!hasErrors) {
      return 0; // No errors detected
    }
    
    // Berlekamp-Massey algorithm to find error locator polynomial
    uint8_t C[2*RS_T+1
