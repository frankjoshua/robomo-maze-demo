#include "Map.h"
#include <Arduino.h>

Map::Map() : grid(nullptr), width(0), height(0) {
    // Initialize with null grid and zero dimensions
}

Map::~Map() {
    // Free allocated memory when object is destroyed
    delete[] grid;
}

void Map::init(int w, int h) {
    // Clean up previous allocation if any
    if (grid) {
        delete[] grid;
    }
    
    width = w;
    height = h;
    
    // Calculate number of bytes needed (1 bit per cell)
    // We need (width * height) bits, which means (width * height + 7) / 8 bytes
    int numBytes = (width * height + 7) / 8;
    
    // Allocate memory for the grid using bytes (unsigned char)
    grid = new unsigned char[numBytes];
    
    // Initialize all bits to 0
    for (int i = 0; i < numBytes; i++) {
        grid[i] = 0;
    }
    
}

void Map::set(int x, int y, int value) {
    // Check if coordinates are within bounds
    if (x >= 0 && x < width && y >= 0 && y < height) {
        int bitPos = y * width + x;
        int bytePos = bitPos / 8;
        int bitOffset = bitPos % 8;
        
        if (value) {
            // Set bit to 1
            grid[bytePos] |= (1 << bitOffset);
        } else {
            // Set bit to 0
            grid[bytePos] &= ~(1 << bitOffset);
        }
        
        // For debugging
        // Serial.print("Setting (");
        // Serial.print(x);
        // Serial.print(",");
        // Serial.print(y);
        // Serial.print(") to ");
        // Serial.print(value);
        // Serial.print(" - bytePos:");
        // Serial.print(bytePos);
        // Serial.print(" bitOffset:");
        // Serial.println(bitOffset);
    }
}

void Map::clear(int x, int y) {
    // Clear means setting to 0
    set(x, y, 0);
}

int Map::get(int x, int y) const {
    // Check if coordinates are within bounds
    if (x >= 0 && x < width && y >= 0 && y < height) {
        int bitPos = y * width + x;
        int bytePos = bitPos / 8;
        int bitOffset = bitPos % 8;
        
        // For debugging
        int result = (grid[bytePos] & (1 << bitOffset)) ? 1 : 0;
        // Serial.print("Getting (");
        // Serial.print(x);
        // Serial.print(",");
        // Serial.print(y);
        // Serial.print(") - bytePos:");
        // Serial.print(bytePos);
        // Serial.print(" bitOffset:");
        // Serial.print(bitOffset);
        // Serial.print(" = ");
        // Serial.println(result);
        
        return result;
    }
    return 0; // Return 0 for out-of-bounds coordinates
}

