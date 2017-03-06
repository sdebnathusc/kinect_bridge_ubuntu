#ifndef _MESSAGES_GEOMETRYMESSAGES_H_
#define _MESSAGES_GEOMETRYMESSAGES_H_

#include <type_traits>
#include <messages/serializable_message.h>
#include <messages/container_messages.h>

class ArithmeticMessageHeader : public MessageHeader
{
public:
    struct Type
    {
        static uint8_t const INT = 0x01;
        static uint8_t const FLOAT = 0x02;
    };

    struct Depth
    {
        static uint8_t const D8 = 0x04;
        static uint8_t const D16 = 0x08;
        static uint8_t const D32 = 0x10;
        static uint8_t const D64 = 0x20;
    };

    struct Sign
    {
        static uint8_t const S = 0x40;
        static uint8_t const U = 0x80;
    };

    uint8_t type_;

    ArithmeticMessageHeader( bool is_float, bool is_signed, uint8_t depth )
    :
        type_( 0x00 )
    {
        if( is_float ) type_ |= Type::FLOAT;
        else type_ |= Type::INT;

        if( depth == 8 ) type_ |= Depth::D8;
        else if( depth == 16 ) type_ |= Depth::D16;
        else if( depth == 32 ) type_ |= Depth::D32;
        else if( depth == 64 ) type_ |= Depth::D64;

        if( is_signed ) type_ |= Sign::S;
        else type_ |= Sign::U;
    }

    ArithmeticMessageHeader( uint8_t type = Type::FLOAT | Depth::D32 | Sign::S )
    :
        type_( type )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << type_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> type_;
    }
};

// meant for integer / floating point types; will fail miserably if std::is_arithmetic<__Data>::value == false
template<class __Data>
class ArithmeticMessage : public SerializableMessageInterface<ArithmeticMessageHeader, __Data>
{
public:
    typedef SerializableMessageInterface<ArithmeticMessageHeader, __Data> _Message;

    ArithmeticMessage( __Data && value = 0 )
    :
        _Message( ArithmeticMessageHeader( std::is_floating_point<__Data>::value, std::is_signed<__Data>::value, sizeof( __Data ) * 8 ), std::forward<__Data>( value ) )
    {
        //
    }

    template<class __Archive>
    void packHeader( __Archive & archive ) const
    {
        this->header_.pack( archive );
    }

    template<class __Archive>
    void packPayload( __Archive & archive ) const
    {
        archive << this->payload_;
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        packHeader( archive );
        packPayload( archive );
    }

    template<class __Archive>
    void unpackHeader( __Archive & archive )
    {
        this->header_.unpack( archive );
    }

    template<class __Archive>
    void unpackPayload( __Archive & archive )
    {
        archive >> this->payload_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        unpackHeader( archive );
        unpackPayload( archive );
    }

    ArithmeticMessage & operator=( __Data const & value )
    {
        this->payload_ = value;
        return *this;
    }

    operator __Data &()
    {
        return this->payload_;
    }

    operator __Data const &() const
    {
        return this->payload_;
    }

    DECLARE_MESSAGE_INFO( ArithmeticMessage )
};

template<class __Data, uint8_t __Dim>
class PointMessage : public ArrayMessage<ArithmeticMessage<__Data>, __Dim>
{
public:
    typedef ArrayMessage<ArithmeticMessage<__Data>, __Dim> _Message;

    template<class... __Args>
    PointMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( PointMessage )
};

typedef PointMessage<float, 3> Point3Message;
typedef PointMessage<float, 4> QuaternionMessage;
typedef PointMessage<float, 6> PoseMessage;
typedef PointMessage<float, 7> PoseQMessage;

#endif // _MESSAGES_GEOMETRYMESSAGES_H_
