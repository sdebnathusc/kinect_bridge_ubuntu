#ifndef _MESSAGES_BINARYCODEC_H_
#define _MESSAGES_BINARYCODEC_H_

#include <sstream>

#include <Poco/MemoryStream.h>
#include <Poco/Timestamp.h>

#include <atomics/binary_stream.h>

#include <messages/codec.h>

template<class __Allocator = std::allocator<char> >
class BinaryCodec : public CodecInterface<CodedMessage<__Allocator> >
{
public:
    typedef CodecInterface<CodedMessage<__Allocator> > _CodecInterface;
    typedef typename _CodecInterface::_CodedMessage _CodedMessage;

    typedef atomics::BinaryOutputStream _Encoder;
    typedef atomics::BinaryInputStream _Decoder;

    typedef std::stringstream _OutputArchive;
    typedef Poco::MemoryInputStream _InputArchive;

    BinaryCodec() = default;

/*
    BinaryCodec( _Encoder && encoder, _Decoder && decoder )
    :
        encoder_( encoder ),
        decoder_( decoder )
    {
        //
    }
*/

    virtual _CodedMessage encode( uint32_t message_id, BinaryMessage<__Allocator> const & binary_message )
    {
//        std::cout << name() << " encoding " << message_type << std::endl;
        // set up output archive
        _OutputArchive archive;
        // set up encoder to write to output archive
        _Encoder encoder( archive, _Encoder::NETWORK_BYTE_ORDER );
        // write byte order
        encoder.writeBOM();

        Poco::Timestamp timer;
//        std::cout << "packing binary message size: " << binary_message.size_ << " via encoder (" << name() << ")" << std::endl;
        // first, have our serializable object pack itself via our encoder type
        binary_message.packPayload( encoder );
        encoder.flush();
        archive.flush();
//        std::cout << "Binary initial message packing took: " << timer.elapsed() / 1000000.0 << std::endl;

        // then, stuff the encoded serializable object into a _CodedMessage
        typename _CodedMessage::_Payload encoded_message( NULL, archive.tellp() );
        // allocate and copy archive into payload

        timer.update();
        encoded_message.unpackPayload( archive );

//        std::cout << "Binary unpacking took: " << timer.elapsed() / 1000000.0 << std::endl;

//        std::cout << "built coded message (encoding: " << name() << " encoded_message: " << message_type << " size (encoded): " << encoded_message.size_ << " size (decoded): " << binary_message.size_ << ")" << std::endl;
        // build our _CodedMessage; fill out the header and copy the payload from the encoder's data
        return _CodedMessage( typename _CodedMessage::_Header( ID(), message_id, binary_message.size_ ), std::move( encoded_message ) );
//        return _CodedMessage( typename _CodedMessage::_Header( name(), message_type, encoded_message.size(), binary_message.size_ ), typename _CodedMessage::_Payload( encoded_message.data(), encoded_message.size() ) );
    }

    virtual BinaryMessage<__Allocator> decode( _CodedMessage const & coded_message )
    {
//        std::cout << name() << " decoding " << coded_message.header_.encoded_message_ << std::endl;

        // set up input archive to read from coded message payload
        _InputArchive archive( coded_message.payload_.data_, coded_message.payload_.size_ );
        // set up decoder to decode from input archive
        _Decoder decoder( archive );
        // read byte order
        decoder.readBOM();

//        std::cout << "unpacking binary message size: " << coded_message.payload_.size_ << std::endl;

        BinaryMessage<__Allocator> binary_message( NULL, coded_message.header_.decoded_size_ );

        // unpack into binary message
        binary_message.unpackPayload( decoder );

        return binary_message;
    }

/*
    // set up a decoder aligned to the payload for the given message
    virtual _Decoder decode( _CodedMessage & message )
    {
        // set up archive to read from encoded payload
        _InputArchive encoded_payload_archive( message.payload_.data_, message_.header_.encoded_size_ );
        // set up decoder to read from encoded payload archive
        return _Decoder( encoded_payload_archive, message.header_.byte_order_ );
    }
*/

    DECLARE_MESSAGE_INFO( BinaryCodecMessage )
};

#endif // _MESSAGES_BINARYCODEC_H_
