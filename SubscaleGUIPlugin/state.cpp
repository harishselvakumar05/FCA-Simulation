#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include <iomanip>
#include <sstream>
    std::array<float, 4> joints;
    std::array<std::string, 4> data;
  void update(){
    std::fstream state;
    std::string line;
    state.open("state.txt");

    if (state.is_open())
      {
        int i = 0;
        while (getline(state,line))
        {
           joints[i] = std::stof(line.substr(line.find(":") + 1)); 
           data[i] = line;
          i++;
        }
      
      state.close();
  }

}



}