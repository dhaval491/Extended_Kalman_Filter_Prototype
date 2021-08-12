#include<fstream>
#include<iostream>
#include<vector>
#include<string>
#include<sstream>
#include<map>
//std::string file_name =  "/home/avii/Documents/SLAM/data/data/sensor_data.dat";
namespace Read_data{

  //map<int,std::vector<int>> get_landmarks(std::string file_name);

std::vector<std::string> split (std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}
 
void Plot_State();

//----------Get odometry data-----------
std::map<int, std::vector<long double>>  get_odometry(std:: string file_name)
{
  std::string Sensor_type = "ODOMETRY";
  std::map<int, std::vector<long double>> Odometry_data;
  
  std::ifstream Data(file_name);
  std::string line; int i = 0;
   while (Data.peek() != EOF) {
   getline(Data,line);
   std::stringstream sstream(line);
   std::vector<std::string> v = split (line," ");
   if(v[0] == Sensor_type)
   
   {
     std::vector<long double> data;
     for(int i = 1;i<4;i++)
     {
       data.push_back(stold(v[i]));
     }
     Odometry_data[i] = data;
     i++;

   }


   }
   //std::cout<< i <<std::endl;
   return Odometry_data;
}

//----------Get Sensor data--------------

std::map<int,std::map<std::string,std::vector<long double>>> get_sensor(std:: string file_name)
{
  std::string Sensor_type = "SENSOR";
  std::map<int,std::map<std::string,std::vector<long double>>> sensor_data;
  std:: string b = "bearing"; std::string r = "range"; std::string id = "id";
  std::ifstream Data(file_name);
  std::string line; int i = -1; bool flag = true;
  std::vector<long double> range;   //v[3]
  std::vector<long double> bearing; //v[2]
  std::vector<long double> ID;
  std::map<std::string,std::vector<long double>> data_map;
  while (Data.peek() != EOF)
  {
    getline(Data,line);
    std::stringstream sstream(line);
    std::vector<std::string> v = split (line," ");
     
    if(v[0] != Sensor_type)
    {
      flag == false;
      //std::cout<<"odo came"<<std::endl;
         if(i>-1)
       {
         //std::cout<<" adding data"<<std::endl;
        data_map[id] = ID;
        data_map[r] = range;
        data_map[b] = bearing;
        sensor_data[i] = data_map;
       }
       range.clear();
       bearing.clear();
       ID.clear();
       data_map.clear();
       //std::cout<< i <<std::endl;
      i++; 
    }
    if(v[0] == Sensor_type)
    {
     // std::cout<<"sensor came"<<std::endl;
    //  if(!flag)
    //  {
    //    if(i>-1)
    //    {
    //      std::cout<<" adding data"<<std::endl;
    //     data_map[id] = ID;
    //     data_map[r] = range;
    //     data_map[b] = bearing;
    //     sensor_data[i] = data_map;
    //    }
    //    range.clear();
    //    bearing.clear();
    //    ID.clear();
    //    data_map.clear();
    //    std::cout<< i <<std::endl;
    //   i++;
    //  }
      //flag == true;
      ID.push_back(stold(v[1]));
      range.push_back(stold(v[3]));
      bearing.push_back(stold(v[2]));
      //std::cout<<"added data"<<std::endl;
      
      
    }
    
  }
        data_map[id] = ID;
        data_map[r] = range;
        data_map[b] = bearing;
        sensor_data[i] = data_map;
        return sensor_data;
   //  if(v[0] == Sensor_type)
  //  //i++;
  //  {
  //    std::vector<long double> data;
  //    for(int i = 1;i<4;i++)
  //    {
  //      data.push_back(stold(v[i]));
  //    }
  //    //sensor_data.push_back(data);

  //  }


}



std::map<int,std::vector<int>> get_landmarks(std::string file_name)
{
  std::map<int,std::vector<int>> land_map;
  std::ifstream Data(file_name);
  std::string line; int i = 0;
   while (Data.peek() != EOF) {
   getline(Data,line);
   std::stringstream sstream(line);
   std::vector<std::string> v = split (line," ");
   std::vector<int> coordinates;
   coordinates.push_back(stoi(v[1]));
   coordinates.push_back(stoi(v[2]));
   land_map[stoi(v[0])] = coordinates;
   }

  return land_map;
}






} //end of namespace Read_data//


// int main()
// {

//   std::map<int,std::vector<long double>> Odometry_data;
//   std::map<int,std::map<std::string,std::vector<long double>>> sensor_data;
//   sensor_data = Read_data::get_sensor(file_name);

//     //  Odometry_data =  Read_data::get_odometry(file_name);
//     std::cout<<sensor_data.size()<<std::endl;



// return 0;
// }


//  ///// /home/avii/Documents/SLAM/data/data/sensor_data.dat  ..... 
// //using namespace std;

// // std::string Sensor_type;
// // std::vector<std::vector<double>> odometry;
