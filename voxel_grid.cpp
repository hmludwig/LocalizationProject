#include <iostream>
#include<fstream>
#include <vector>
#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <sstream>

using namespace std;
template <class InIt, class OutIt>
void Split(InIt begin, InIt end, OutIt splits)
{
    InIt current = begin;
    while (begin != end)
    {
        if (*begin == ',')
        {
            *splits++ = std::string(current,begin);
            current = ++begin;
        }
        else
            ++begin;
    }
    *splits++ = std::string(current,begin);
}

void read()
{
    ifstream fin;
    float Frame, x, y, z, roll, pitch, yaw;
    string line;
    vector<float>  m_data;
    // Open an existing file
    fin.open("/home/harald/LocalizationProject/dataset/ground_truth.csv");

    vector<string> strs;
    vector<std::string> m_vecFields;
    while(!fin.eof()){
          fin >> line;
          //boost::split(strs, line, boost::is_any_of(","));
          cout << line << std::endl;
          //Split(line.begin(), line.end(), back_inserter(m_vecFields));
          //cout << m_vecFields[0] << endl;
          
    }
}
int main()
{
    read();
    return 0;
}