#ifndef INVALID_CLOUD_POINTER_H
#define INVALID_CLOUD_POINTER_H

#include <exception>
#include <stdexcept>

namespace cos_lib
{
    namespace except
    {
        class invalid_cloud_pointer : public std::invalid_argument
        {
        private:
            const char *_what = "Invalid cloud pointer argument.";

        public:
            invalid_cloud_pointer() : std::invalid_argument(_what) { }
            virtual const char *what() const throw() { return _what; }
        };
    }
}

#endif // INVALID_CLOUD_POINTER_H
