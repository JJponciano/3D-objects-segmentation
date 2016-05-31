#ifndef INVALID_PATH_H
#define INVALID_PATH_H

#include <exception>
#include <stdexcept>


class invalid_path : public std::invalid_argument
{
private:
    const char *_what = "Invalid path argument.";

public:
    invalid_path() : std::invalid_argument(_what) { }
    virtual const char *what() const throw() { return _what; }
};

#endif // INVALID_PATH_H
