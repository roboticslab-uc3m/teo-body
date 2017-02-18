// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Playback.hpp"

namespace teo
{

// -----------------------------------------------------------------------------

bool Playback::open(yarp::os::Searchable& config)
{
    std::string fileName = config.check("file",yarp::os::Value("in.txt"),"file name").asString();

    file.open(fileName.c_str());
    if( ! file.is_open() )
    {
          printf("Not able to open file.\n");
          return false;
    }

    std::vector<double> doublesOnFileLine;

    while( this->parseFileLine(doublesOnFileLine) )
    {
        if ( doublesOnFileLine.size() == 0 ) continue;

        doublesOnFile.push_back( doublesOnFileLine );
    }

    doublesOnFileIter = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool Playback::close()
{
    file.close();
    return true;
}

// -----------------------------------------------------------------------------

bool Playback::getNumRows(int* num)
{
    *num = doublesOnFile.size();

    return true;
}

// -----------------------------------------------------------------------------

bool Playback::getNext(std::vector<double>& row)
{
    if( doublesOnFileIter >= doublesOnFile.size() )
        return false;

    row = doublesOnFile[doublesOnFileIter];

    doublesOnFileIter++;

    return true;
}

// -----------------------------------------------------------------------------

bool Playback::parseFileLine(std::vector<double>& doublesOnFileLine)
{
    doublesOnFileLine.clear();

    if( file.eof() )
        return false;

    std::string s;
    getline(file, s);
    std::stringstream ss(s);

    double d;
    while (ss >> d)
        doublesOnFileLine.push_back(d);

    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
