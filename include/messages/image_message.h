#ifndef _MESSAGES_IMAGEMESSAGE_H_
#define _MESSAGES_IMAGEMESSAGE_H_

#include <messages/binary_message.h>

class ImageMessageHeader : public MessageHeader
{
public:
    uint16_t width_;
    uint16_t height_;
    uint8_t num_channels_;
    uint8_t pixel_depth_;
    std::string encoding_;

    ImageMessageHeader( uint16_t width = 0, uint16_t height = 0, uint8_t num_channels = 0, uint8_t pixel_depth = 0, std::string const & encoding = "" )
    :
        width_( width ),
        height_( height ),
        num_channels_( num_channels ),
        pixel_depth_( pixel_depth ),
        encoding_( encoding )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
//        std::cout << "packing ImageMessageHeader" << std::endl;
        archive << width_;
        archive << height_;
        archive << num_channels_;
        archive << pixel_depth_;
        archive << encoding_;

    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
//        std::cout << "unpacking ImageMessageHeader" << std::endl;
        archive >> width_;
        archive >> height_;
        archive >> num_channels_;
        archive >> pixel_depth_;
        archive >> encoding_;

        std::cout << static_cast<int>( width_ ) << std::endl;
        std::cout << static_cast<int>( height_ ) << std::endl;
        std::cout << static_cast<int>( num_channels_ ) << std::endl;
        std::cout << static_cast<int>( pixel_depth_ ) << std::endl;
        std::cout << encoding_ << std::endl;
    }
};

template<class __Allocator = std::allocator<char> >
class ImageMessage : public SerializableMessageInterface<ImageMessageHeader, BinaryMessage<__Allocator> >
{
public:
    typedef SerializableMessageInterface<ImageMessageHeader, BinaryMessage<__Allocator> > _Message;

    template<class... __Args>
    ImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // use default packing/unpacking from SerializableMessage

    DECLARE_MESSAGE_INFO( ImageMessage )
};

#endif // _MESSAGES_IMAGEMESSAGE_H_
