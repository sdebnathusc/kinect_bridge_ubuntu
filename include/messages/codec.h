#ifndef _MESSAGES_CODEC_H_
#define _MESSAGES_CODEC_H_

#include <messages/binary_message.h>
#include <messages/container_messages.h>

class CodedMessageHeader : public RecursiveMessageHeader
{
public:
    uint32_t encoding_;
    uint32_t decoded_size_;

    CodedMessageHeader( uint32_t encoding = 0, uint32_t payload_id = 0, uint32_t decoded_size = 0 )
    :
        RecursiveMessageHeader( payload_id ),
        encoding_( encoding ),
        decoded_size_( decoded_size )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << encoding_;
        RecursiveMessageHeader::pack( archive );
        archive << decoded_size_;

    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> encoding_;
        RecursiveMessageHeader::unpack( archive );
        archive >> decoded_size_;
    }

/*
    virtual std::string const & name() const
    {
        static std::string const name( "CodedMessageHeader" );
        return name;
    }
*/
};

// a coded message always operates on a binary payload
template<class __Allocator = std::allocator<char> >
class CodedMessage : public SerializableMessageInterface<CodedMessageHeader, BinaryMessage<__Allocator> >
{
public:
    typedef SerializableMessageInterface<CodedMessageHeader, BinaryMessage<__Allocator> > _Message;
    typedef __Allocator _Allocator;

    template<class... __Args>
    CodedMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // use default pack/unpack from SerializableMessage

    DECLARE_MESSAGE_INFO( CodedMessage )
};

template<class __CodedMessage, class __Allocator = std::allocator<char> >
class EncoderInterface : public NamedInterface
{
public:
    virtual __CodedMessage encode( uint32_t message_id, char const * data, uint32_t size )
    {
        return encode( message_id, BinaryMessage<__Allocator>( data, size ) );
    }

    virtual __CodedMessage encode( uint32_t message_id, BinaryMessage<__Allocator> const & message ) = 0;
};

template<class __CodedMessage, class __Allocator = std::allocator<char> >
class DecoderInterface : public NamedInterface
{
public:
    virtual char * decode( __CodedMessage const & message, uint32_t & size )
    {
        BinaryMessage<__Allocator> binary_message( decode( message ) );
        size = binary_message.size_;
        return binary_message.data_;
    }

    virtual BinaryMessage<__Allocator> decode( __CodedMessage const & message ) = 0;
};

template<class __CodedMessage>
class CodecInterface : public virtual EncoderInterface<__CodedMessage>, public virtual DecoderInterface<__CodedMessage>
{
public:
    typedef __CodedMessage _CodedMessage;
    typedef typename __CodedMessage::_Allocator _Allocator;
};

#endif // _MESSAGES_CODEC_H_
