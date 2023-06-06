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
        return;
    }

    string line;
    const int MAX_WORDS = 12;
    int lineNum = 0;
    string* words[MAX_WORDS];
    while(getline(inputFile, line))
    {
        int numWords = 0;
        istringstream iss(line);
        string word;
        if (lineNum == numLines)
        {
            cout << "++++++++++++++++++++++++++++++++++++++" << endl;
            while (iss >> word && numWords < MAX_WORDS)
            {
                words[numWords] = word;
                cout << "words[" << numWords << "] = " << words[numWords] << endl;
                numWords++;
            }
        }
        // cout << "line " << lineNum << ": " << words[] << endl;
        lineNum++;
    }
    cout << "++++++++++++++++++++++++++++++++++++++" << endl;
    inputFile.close();

    return words;
}

int main()
{
    string filename = "/home/vboxuser/catkin_ws/src/px4_ground/mission/mission_spiral.txt";
    int numLines = 1;
    string* words = printFile(filename, numLines);
    if (words != nullptr)
    {
        // do something with words
        delete[] words; // free the memory when done
    }
    return 0;
}