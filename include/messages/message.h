#ifndef _MESSAGES_MESSAGE_H_
#define _MESSAGES_MESSAGE_H_

#include <messages/typedefs.h>
#include <utility>

class MessageComponent
{
    //
};

class MessageHeader : public MessageComponent
{
    //
};

class MessagePayload : public MessageComponent
{
    //
};

template<class __Header, class __Payload>
class Message : public MessageComponent
{
public:
    __Header header_;
    __Payload payload_;

    Message() = default;

    Message( __Header && header, __Payload && payload )
    :
        header_( std::forward<__Header>( header ) ),
        payload_( std::forward<__Payload>( payload ) )
    {
//        std::cout << "Message forward components constructor (" << this->vName() << ")" << std::endl;
    }

/*
    template<class __OtherHeader, class __OtherPayload>
    Message( Message<__OtherHeader, __OtherPayload> && other )
    :
        header_( std::move( other.header_ ) ),
        payload_( std::move( other.payload_ ) )
    {
//        std::cout << "Message move components constructor (" << this->vName() << ")" << std::endl;
    }
*/

/*
    template<class __OtherHeader, class __OtherPayload>
    Message( Message<__OtherHeader, __OtherPayload> const & other )
    :
        header_( other.header_ ),
        payload_( other.payload_ )
    {
        std::cout << "Message copy components constructor (" << this->name() << ")" << std::endl;
    }

    Message<__Header, __Payload> & operator=( Message<__Header, __Payload> const & other )
    {
        if( this != &other )
        {

        }
        return *this;
    }
*/
};

class EmptyHeader : public MessageHeader
{
public:
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        //
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        //
    }
};

class EmptyPayload : public MessagePayload
{
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        //
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        //
    }
};


#endif // _MESSAGES_MESSAGE_H_
