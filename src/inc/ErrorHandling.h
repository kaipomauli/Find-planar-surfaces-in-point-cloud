#ifndef _ICON_ERROR_HANDLING_
#define _ICON_ERROR_HANDLING_

// Include the EthernetCamera camera header
#include "icamerasystem.h"
#include <exception>
#include <sstream>

class icon_exception : public std::exception
{
public:
    icon_exception(icon::ErrorCode code, const char *file = NULL, unsigned int line = 0) : mCode(code)
    {
        std::stringstream oss;
        if(file)
        {
            oss << file << "(" << line << ") ";
        }
        oss << " Icon Error " << mCode << " occurred.";
        mString = oss.str();
    }

    const char * what() const
    {
        return mString.c_str();
    }

private:
    icon::ErrorCode mCode;
    std::string mString;
};

#define CHECK_ERROR(res) \
    check_error(res, __FILE__, __LINE__)

void check_error(icon::ErrorCode res, const char *file = NULL, unsigned int line = 0)
{
    if(res != icon::E_ALL_OK)
    {
        throw icon_exception(res, file, line);
    }
}

#endif