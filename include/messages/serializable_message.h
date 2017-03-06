#ifndef _MESSAGES_SERIALIZABLEMESSAGE_H_
#define _MESSAGES_SERIALIZABLEMESSAGE_H_

#include <string>
//#include <memory>
#include <iostream>

#include <Poco/MD5Engine.h>

#include <messages/message.h>
#include <messages/macros.h>

template<class __Message>
static uint32_t MessageID()
{
    static Poco::MD5Engine md5_engine;
    md5_engine.reset();
    md5_engine.update( __Message::name() );
    Poco::DigestEngine::Digest const digest = md5_engine.digest();

    /* note that we only use the first 4 of 16 bytes of the digest here */
    return *reinterpret_cast<uint32_t const *>( digest.data() );
}

class NamedInterface
{
public:
    virtual std::string const & vName() const = 0;
};

class SerializableInterface : public NamedInterface
{
public:
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        // do nothing
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        // do nothing
    }
};

template<class __Header, class __Payload>
class SerializableMessageInterface : public Message<__Header, __Payload>, public SerializableInterface
{
public:
    typedef Message<__Header, __Payload> _Message;
    typedef __Header _Header;
    typedef __Payload _Payload;
//    typedef std::shared_ptr<_Message> _Ptr;

    template<class... __Args>
    SerializableMessageInterface( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

/*
    template<class __Archive>
    void packHeader( __Archive & archive ) const
    {
        // by default, just ask header to pack
        this->header_.pack( archive );
    }

    template<class __Archive>
    void packPayload( __Archive & archive ) const
    {
        // by default, just ask payload to pack
        this->payload_.pack( archive );
    }
*/

    template<class __Archive>
    void pack( __Archive & archive )
    {
//        std::cout << "packing " << this->vName() << " via SerializableMessageInterface" << std::endl;
        this->header_.pack( archive );
        this->payload_.pack( archive );
    }

    template<class __Message, class __Archive>
    void pack( __Message & message, __Archive & archive )
    {
//        std::cout << "delegating packing " << message.name() << " via SerializableMessageInterface" << std::endl;
        message.packHeader( archive );
        message.packPayload( archive );
    }

/*
    template<class __Archive>
    void unpackHeader( __Archive & archive )
    {
        // by default, just ask header to unpack
        this->header_.unpack( archive );
    }

    template<class __Archive>
    void unpackPayload( __Archive & archive )
    {
        // by default, just ask payload to unpack
        this->payload_.unpack( archive );
    }

*/
    template<class __Archive>
    void unpack( __Archive & archive )
    {
//        std::cout << "unpacking " << this->vName() << " via SerializableMessageInterface" << std::endl;
        this->header_.unpack( archive );
        this->payload_.unpack( archive );
    }

    template<class __Message, class __Archive>
    void unpack( __Message & message, __Archive & archive )
    {
//        std::cout << "delegating unpacking " << message.name() << " via SerializableMessageInterface" << std::endl;
        message.unpackHeader( archive );
        message.unpackPayload( archive );
    }

    template<class __Base, class __Archive>
    void packAs( __Archive & archive )
    {
//        __Base::pack( archive );
        static_cast<__Base *>( this )->pack( archive );
    }

    template<class __Base, class __Archive>
    void unpackAs( __Archive & archive )
    {
        static_cast<__Base *>( this )->unpack( archive );
    }
};

class EmptyMessage : public SerializableMessageInterface<EmptyHeader, EmptyPayload>
{
public:
    DECLARE_MESSAGE_INFO( EmptyMessage )
};

#endif // _MESSAGES_SERIALIZABLEMESSAGE_H_
