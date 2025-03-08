#ifndef MAP_H
#define MAP_H

class Map {
private:
    unsigned char* grid;  // Using bytes instead of ints, each bit represents a cell
    int width;      // Width of the map
    int height;     // Height of the map
    
public:
    // Constructor/Destructor
    Map(unsigned char* grid, int width, int height);
    ~Map();
    
    // Initialize map with given dimensions
    void init(int width, int height);
    
    // Set value at x,y position
    void set(int x, int y, int value);
    
    // Clear value at x,y position (set to 0)
    void clear(int x, int y);
    
    // Get value at x,y position
    int get(int x, int y) const;
};

#endif // MAP_H