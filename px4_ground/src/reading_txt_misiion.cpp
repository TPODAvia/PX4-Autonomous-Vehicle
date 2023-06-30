#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;
string* printFile(string filename, int numLines)
{
    ifstream inputFile(filename);

    if (!inputFile)
    {
        cout << "Error opening file" << endl;
        return nullptr;
    }

    string line;
    const int MAX_WORDS = 12;
    int lineNum = 0;
    string* words = new string[MAX_WORDS];
    while(getline(inputFile, line))
    {
        int numWords = 0;
        istringstream iss(line);
        string word;
        if (lineNum == numLines)
        {
            while (iss >> word && numWords < MAX_WORDS)
            {
                words[numWords] = word;
                // cout << "words[" << numWords << "] = " << words[numWords] << endl;
                numWords++;
            }
        }
        lineNum++;
    }
    inputFile.close();

    return words;
}

bool command_execute(string* words)
{
    if (words[2] == "3")
    {
        int num = stoi(words[3]);
        switch (num)
        {
            case 16:
                cout << "case 1" << endl;
                // Do wapoint mission
                return true;
            case 20:
                cout << "case 2" << endl;
                // Do return to launch
                return true;
            case 21:
                cout << "case 3" << endl;
                // Do landing
                return true;
            case 22:
                cout << "case 4" << endl;
                // Do take off
                return true;
            default:
                cout << "Error: Commands is wrong" << endl;
                return false;
        }
    }
    else
    {
        cout << "Error: The frame type is incorect" << endl;
        return false;
    }

}

int main()
{
    string filename = "/home/vboxuser/catkin_ws/src/px4_ground/mission/mission_spiral.txt";
    int numLines = 1;
    string* words = printFile(filename, numLines);
    command_execute(words);

    if (words != nullptr)
    {
        // do something with words
        delete[] words; // free the memory when done
    }
    return 0;
}