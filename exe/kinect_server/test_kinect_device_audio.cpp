#include <iostream>
#include <fstream>
#include <thread>

#include <kinect_common/kinect_device.h>
#include <messages/wav_audio_message.h>

// will accept any message type derived from AudioMessage
typedef KinectAudioMessage<WAVAudioMessage<> > _AudioMsg;

int main()
{
    std::ofstream output_stream1( "test_kinect_audio.bin", std::ios::binary );
    std::ofstream output_stream2( "test_kinect_audio.wav", std::ios::binary );
    std::ofstream output_stream3( "test_kinect_audio.pak", std::ios::binary );

    KinectDevice kinect_device;
    while( true )
    {
        try
        {
            kinect_device.initialize();
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );
            continue;
        }
    }

    _AudioMsg output_message;

    std::vector<std::shared_ptr<_AudioMsg> > output_message_ptrs;

    // record about 1 second of audio
    while( output_message.header_.num_samples_ < 16000 )
    {
        try
        {
            std::shared_ptr<_AudioMsg> output_message_ptr;
            kinect_device.pullAudio( output_message_ptr );

            output_message.payload_.size_ += output_message_ptr->payload_.size_;
            output_message.header_.num_samples_ += output_message_ptr->header_.num_samples_;

            std::cout << output_message_ptr->beam_angle_ << " (" << output_message_ptr->beam_angle_confidence_ << ")" << std::endl;

            output_message_ptrs.push_back( output_message_ptr );
        }
        catch( KinectException & e )
        {
//            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
            continue;
        }
    }

    // combine all messages into one message

    output_message.header_.num_channels_ = output_message_ptrs.front()->header_.num_channels_;
    output_message.header_.sample_depth_ = output_message_ptrs.front()->header_.sample_depth_;
    output_message.header_.sample_rate_ = output_message_ptrs.front()->header_.sample_rate_;
    output_message.header_.encoding_ = output_message_ptrs.front()->header_.encoding_;

    output_message.payload_.allocate();

    uint32_t payload_offset = 0;
    for( auto output_message_it = output_message_ptrs.cbegin(); output_message_it != output_message_ptrs.cend(); ++output_message_it )
    {
        auto & output_message_ptr = *output_message_it;

        std::memcpy( output_message.payload_.data_ + payload_offset, output_message_ptr->payload_.data_, output_message_ptr->payload_.size_ );

        payload_offset += output_message_ptr->payload_.size_;
    }

    std::cout << "packing raw audio" << std::endl;

    // first, write raw audio
    output_message.payload_.pack( output_stream1 );

    std::cout << "packing wav audio" << std::endl;

    // now, encode as wav
    output_message.packAs<WAVAudioMessage<> >( output_stream2 );

    std::cout << "packing kinect audio" << std::endl;

    // save entire message including timestamp
    output_message.pack( output_stream3 );

    output_stream1.close();
    output_stream2.close();
    output_stream3.close();

    return 0;
}
