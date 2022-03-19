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

void write(std::string tag, float val){
    std::ostringstream out;
    out.precision(3);
    std::string d = ":";
    std::ofstream state;
    std::string line;
    state.open("state.txt");
    if (state.is_open()){
      for(int x = 0; x < 4; x++)
      {
        if((data[x].find(tag))){
            out << std::fixed << data[x] + "\n";
        } else {
            out << std::fixed << tag + ":" << val << "\n";
        }
    }
    state << out.str();  
    state.close();
    update(); 

  }
}

int main(){
  update();
for(float x = 0; x < 20; x += 0.1){
  write("left_aileron", x);
  write("right_aileron", -x);
  std::cout << x << std::endl;
} 
for(int x = 0; x < joints.size(); x++){
    std::cout << joints[x] << std::endl;
}
}

