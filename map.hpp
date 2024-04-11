#pragma once

#include <array>
#include <boost/assert.hpp>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <fstream>
#include <vector>
#include "structs.hpp"
#include "constants.hpp"



inline bool isOccupied(char c){
    switch (c){
        case '.':
            return false;
        case 'G':
            return false;
        case 'S':
            return false;
        case '@':
            return true;
        case 'O':
            return true;
        case 'T':
            return true;
        case 'W':
            return true;
        default :
            std::cerr << "ERROR: unknown map symbol: '" << c << "'\n"; 
            exit(EXIT_FAILURE);
    }
}

struct Map{
    unsigned int height;
    unsigned int width;
    std::size_t size;
    std::vector<bool> occupancy;

    Map(std::string filename){
        std::ifstream mapfile;
        std::string line;
        mapfile.open(filename);

        mapfile >> line;
        assert(line == "type");
        mapfile >> line;
        assert(line == "octile");


        mapfile >> line;
        assert(line == "height");
        mapfile >> height;

        mapfile >> line;
        assert(line == "width");
        mapfile >> width;

        size = width*height;
        occupancy.resize(size);

        mapfile >> line;
        assert(line == "map");
        char c;
        std::size_t ind = 0; 
        while(mapfile.get(c) && ind < size){
            if (c == '\n'){
                mapfile.get(c);
            }
            occupancy[ind++] = isOccupied(c);
        }
        mapfile.close();
        assert(ind == size);
    }

    constexpr void checkBounds(int x, int y) const{
        (void)x;
        (void)y;
        assert (x >= 0 && (unsigned int)x < width);
        assert (y >= 0 && (unsigned int)y < height);
    }
    constexpr bool inBounds(int x, int y) const{
        return (x >= 0 && (unsigned int)x < width) && (y >= 0 && (unsigned int)y < height);
    }

    inline std::size_t getIndex(int x, int y) const{
        checkBounds(x, y);
        std::size_t retval = x + width*y; 
        assert(retval <= std::numeric_limits<std::uint32_t>::max());
        return retval;
    }

    inline Location index2Location(std::size_t index) const{
        int x = index % width;
        int y = index / width;
        return Location(x, y);
    }

    inline std::size_t getIndex(Location s) const{
        return getIndex(s.x(), s.y()); 
    }

    inline bool isBlocked(int x, int y) const{
        return occupancy[getIndex(x, y)];
    }

    inline bool isSafe(int x, int y) const{
        return !isBlocked(x, y);
    }

    inline std::vector<Location> safeLocs() const{
        std::vector<Location> retval;
        for(std::size_t i = 0; i < size; i++){
            if(!occupancy[i]){
                retval.emplace_back(index2Location(i));
            }
        }
        return retval;
    } 

    void debug()const{
        for (uint j = 0; j < height; j++){
            for (uint i = 0; i < width; i++){
                std::cout << isSafe(i, j);
            }
            std::cout << "\n";
        }
    }
};

struct Scenario{
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    double optimal_length;

    Scenario(std::ifstream& line){
        std::string junk;
        for(int i = 0; i < 4; i++){
            line >> junk;
        }
        line >> start_x;
        line >> start_y;
        line >> goal_x;
        line >> goal_y;
        line >> optimal_length;
    }

    static inline std::vector<Scenario> read_scenarios(const std::string& filepath){
        std::ifstream f(filepath);
        std::string word;
        f >> word;
        assert(word == "version");
        f >> word;
        f.get();
        std::vector<Scenario> retval;
        std::cout << f.peek() << std::endl;
        while(f && std::isdigit(f.peek())){
            retval.emplace_back(f);
            f.get();
        }
        return retval;
    }

    void debug() const{
        std::cout << start_x << " " << start_y << " " << goal_x << " " << goal_y << " " << optimal_length << std::endl; 
    }

    
};
