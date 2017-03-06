#ifndef _MESSAGES_MESSAGECODER_H_
#define _MESSAGES_MESSAGECODER_H_

#include <sstream>

#include <Poco/MemoryStream.h>
#include <Poco/Timestamp.h>

#include <atomics/binary_stream.h>

#include <messages/codec.h>

// packs given message to binary, passes to given codec, wraps in CodedMessage
template<class __Codec>
class MessageCoder : public __Codec
{
public:
    typedef typename __Codec::_Allocator _Allocator;
    typedef typename __Codec::_CodedMessage _CodedMessage;

    typedef std::stringstream _OutputArchive;
    typedef Poco::MemoryInputStream _InputArchive;

    typedef atomics::BinaryOutputStream _BinaryWriter;
    typedef atomics::BinaryInputStream _BinaryReader;

    template<class... __Args>
    MessageCoder( __Args&&... args )
    :
        __Codec( std::forward<__Args>( args )... )
    {
        //
    }

    template<class __Serializable>
    _CodedMessage encode( __Serializable & serializable )
    {
        _OutputArchive archive;
        // set up binary writer to write to output archive
        _BinaryWriter binary_writer( archive, _BinaryWriter::NETWORK_BYTE_ORDER );

        Poco::Timestamp timer;
//        std::cout << "MessageCoder packing serializable to binary" << std::endl;
        // pack serializable to binary
        serializable.pack( binary_writer );
        binary_writer.flush();
        archive.flush();

//        std::cout << "Generic initial message packing took: " << timer.elapsed() / 1000000.0 << std::endl;

        timer.update();

        // encode using __Codec
        BinaryMessage<_Allocator> binary_message( NULL, archive.tellp() );
        binary_message.unpackPayload( archive );

//        std::cout << "Generic unpacking took: " << timer.elapsed() / 1000000.0 << std::endl;
//        std::cout << "MessageCoder packed serializable to binary; size: " << binary_message.size_ << std::endl;

//        std::cout << "MessageCoder encoding via " << __Codec::name() << std::endl;
        return __Codec::encode( serializable.ID(), std::move( binary_message ) );
    }

    template<class __Base, class __Serializable>
    _CodedMessage encodeAs( __Serializable & serializable )
    {
        _OutputArchive archive;
        // set up binary writer to write to output archive
        _BinaryWriter binary_writer( archive, _BinaryWriter::NETWORK_BYTE_ORDER );

//        std::cout << "MessageCoder packing serializable to binary" << std::endl;
        // pack serializable to binary
        serializable.template packAs<__Base>( binary_writer );
        binary_writer.flush();
        archive.flush();

        // encode using __Codec
        BinaryMessage<_Allocator> binary_message( NULL, archive.tellp() );
        binary_message.unpackPayload( archive );
//        std::cout << "MessageCoder packed serializable to binary; size: " << binary_message.size_ << std::endl;

//        std::cout << "MessageCoder encoding via " << __Codec::name() << std::endl;
        return __Codec::encode( serializable.ID(), std::move( binary_message ) );
    }

    template<class __Serializable>
    void decode( __Serializable & serializable, _CodedMessage const & coded_message )
    {
//        std::cout << "MessageCoder decoding via " << __Codec::name() << std::endl;
        // decode into BinaryMessage using __Codec
        BinaryMessage<_Allocator> binary_message = __Codec::decode( coded_message );

        // set up archive to read from decoded binary message
        _InputArchive archive( binary_message.data_, binary_message.size_ );

        // set up binary reader to read from input archive
        _BinaryReader binary_reader( archive, _BinaryReader::NETWORK_BYTE_ORDER );

//        std::cout << "MessageCoder unpacking serializable from binary; size: " << binary_message.size_ << std::endl;
        // unpack into __Serializable using binary_reader
        serializable.unpack( binary_reader );
    }

    template<class __Base, class __Serializable>
    void decodeAs( __Serializable & serializable, _CodedMessage const & coded_message )
    {
//        std::cout << "MessageCoder decoding via " << __Codec::name() << std::endl;
        // decode into BinaryMessage using __Codec
        BinaryMessage<_Allocator> binary_message = __Codec::decode( coded_message );

        // set up archive to read from decoded binary message
        _InputArchive archive( binary_message.data_, binary_message.size_ );

        // set up binary reader to read from input archive
        _BinaryReader binary_reader( archive, _BinaryReader::NETWORK_BYTE_ORDER );

//        std::cout << "MessageCoder unpacking serializable from binary; size: " << binary_message.size_ << std::endl;
        // unpack into __Serializable using binary_reader
        serializable.template unpackAs<__Base>( binary_reader );
    }

    template<class __Serializable>
    __Serializable decode( _CodedMessage const & coded_message )
    {
        __Serializable serializable;
        decode( serializable, coded_message );
        return serializable;
    }

    template<class __Base, class __Serializable>
    __Serializable decodeAs( _CodedMessage const & coded_message )
    {
        __Serializable serializable;
        decodeAs<__Base>( serializable, coded_message );
        return serializable;
    }

    // decode CodedMessage from BinaryMessage storage
    _CodedMessage decode( BinaryMessage<_Allocator> const & raw_coded_message )
    {
        Poco::MemoryInputStream raw_input_stream( raw_coded_message.data_, raw_coded_message.size_ );
        _BinaryReader binary_reader( raw_input_stream, _BinaryReader::NETWORK_BYTE_ORDER );

        _CodedMessage coded_message;

        coded_message.unpack( binary_reader );

        return coded_message;
    }

/*
    // special function to just peek at the header so others can decide if they need to unpack the rest of the message
    template<class __Serializable>
    typename __Serializable::_Header peekHeader( __Serializable const & serializable )
    {
        //
    }
*/
};

#endif // _MESSAGES_MESSAGECODER_H_
