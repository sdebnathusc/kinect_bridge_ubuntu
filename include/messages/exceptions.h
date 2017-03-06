#ifndef _MESSAGES_EXCEPTIONS_H_
#define _MESSAGES_EXCEPTIONS_H_

#include <stdexcept>

namespace messages
{

class MessageException : public std::runtime_error
{
public:
    template<class... __Args>
    MessageException( __Args&&... args )
    :
        std::runtime_error( std::forward<__Args>( args )... )
    {
        //
    }
};

} // messages

#endif //_MESSAGES_EXCEPTIONS_H_
