#ifndef _MESSAGES_AUDIOMESSAGE_H_
#define _MESSAGES_AUDIOMESSAGE_H_

#include <messages/binary_message.h>

template<class __NumSamplesType = uint32_t, class __NumChannelsType = uint8_t, class __SampleRateType = uint16_t>
class AudioMessageHeader : public MessageHeader
{
public:
    __NumSamplesType num_samples_;
    __NumChannelsType num_channels_;
    uint8_t sample_depth_;
    __SampleRateType sample_rate_;
    std::string encoding_;

    AudioMessageHeader( __NumSamplesType num_samples = 0, __NumChannelsType num_channels = 0, uint8_t sample_depth = 0, __SampleRateType sample_rate = 0, std::string encoding = "" )
    :
        num_samples_( num_samples ),
        num_channels_( num_channels ),
        sample_depth_( sample_depth ),
        sample_rate_( sample_rate ),
        encoding_( encoding )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << num_samples_;
        archive << num_channels_;
        archive << sample_depth_;
        archive << sample_rate_;
        archive << encoding_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> num_samples_;
        archive >> num_channels_;
        archive >> sample_depth_;
        archive >> sample_rate_;
        archive >> encoding_;
    }

};

template<class __Header = AudioMessageHeader<>, class __Allocator = std::allocator<char> >
class AudioMessage : public SerializableMessageInterface<__Header, BinaryMessage<__Allocator> >
{
public:
    typedef SerializableMessageInterface<__Header, BinaryMessage<__Allocator> > _Message;

    template<class... __Args>
    AudioMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( AudioMessage )
};

#endif // _MESSAGES_AUDIOMESSAGE_H_
