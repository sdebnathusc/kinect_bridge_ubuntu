#ifndef _MESSAGES_UTILITYMESSAGES_H_
#define _MESSAGES_UTILITYMESSAGES_H_

#include <messages/binary_message.h>
#include <Poco/MD5Engine.h>

class TimeStampMessage : public SerializableInterface
{
public:
    uint64_t stamp_;

    TimeStampMessage()
    :
        stamp_( 0 )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
//      stamp_ = now();
        archive << stamp_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> stamp_;
    }

    DECLARE_MESSAGE_INFO( TimeStampMessage )
};

class SequenceMessage : public SerializableInterface
{
public:
    uint32_t sequence_;

    SequenceMessage()
    :
        sequence_( 0 )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        sequence_ ++;
        archive << sequence_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> sequence_;
    }

    DECLARE_MESSAGE_INFO( SequenceMessage )
};

class ChecksumMessageHeader : public MessageHeader
{
public:
    std::string engine_;
    std::string checksum_;

    ChecksumMessageHeader( std::string const & engine = "", std::string const & checksum = "" )
    :
        engine_( engine ),
        checksum_( checksum )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << engine_;
        archive << checksum_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> engine_;
        archive >> checksum_;
    }
};

template<class __Engine, class __Allocator = std::allocator<char> >
class ChecksumMessage : public SerializableMessageInterface<ChecksumMessageHeader, BinaryMessage<__Allocator> >
{
public:
    typedef SerializableMessageInterface<ChecksumMessageHeader, BinaryMessage<__Allocator> > _Message;

    __Engine engine_;

    template<class... __Args>
    ChecksumMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        engine_.update( this->payload_.data_, this->payload_.size_ );
        this->header_.checksum_ = Poco::DigestEngine::digestToHex( engine_.digest() );

        this->header_.pack( archive );
        this->payload_.pack( archive );
    }

    DECLARE_MESSAGE_INFO( ChecksumMessage )

    // default unpack from SerializableMessageInterface
};

template<class __Allocator = std::allocator<char> >
class MD5ChecksumMessage : public ChecksumMessage<Poco::MD5Engine, __Allocator>
{
public:
    typedef ChecksumMessage<Poco::MD5Engine, __Allocator> _Message;

    template<class... __Args>
    MD5ChecksumMessage( __Args&&... args )
    :
        _Message( ChecksumMessageHeader( "MD5" ), BinaryMessage<__Allocator>( std::forward<__Args>( args )... ) )
    {
        //
    }

    // default pack from ChecksumMessage
    // default unpack from SerializableMessageInterface
    // name defined in ChecksumMessage
};

#endif // _MESSAGES_UTILITYMESSAGES_H_
