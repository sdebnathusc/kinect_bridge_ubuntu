#include <iostream>
#include <thread>
#include <deque>
#include <memory>
#include <sstream>
#include <fstream>

// we have to include this before any Poco code (or any code that includes Poco code) otherwise windows speech API will go full retard
#include <kinect_common/kinect_device.h>

#include <Poco/Runnable.h>
#include <Poco/TimeStamp.h>
#include <Poco/ThreadPool.h>

#include <atomics/wrapper.h>
#include <atomics/print.h>
#include <atomics/binary_stream.h>

#include <messages/message_coder.h>
#include <messages/binary_codec.h>
#include <messages/gzip_codec.h>

#include <messages/png_image_message.h>
#include <messages/wav_audio_message.h>

typedef KinectColorImageMessage<PNGImageMessage<> > _ColorImageMsg;
typedef std::shared_ptr<_ColorImageMsg> _ColorImageMsgPtr;

typedef KinectDepthImageMessage<PNGImageMessage<> > _DepthImageMsg;
typedef std::shared_ptr<_DepthImageMsg> _DepthImageMsgPtr;

typedef KinectInfraredImageMessage<PNGImageMessage<> > _InfraredImageMsg;
typedef std::shared_ptr<_InfraredImageMsg> _InfraredImageMsgPtr;

typedef KinectAudioMessage<WAVAudioMessage<> > _AudioMsg;
typedef std::shared_ptr<_AudioMsg> _AudioMsgPtr;

typedef KinectBodiesMessage _BodiesMsg;
typedef std::shared_ptr<_BodiesMsg> _BodiesMsgPtr;

typedef KinectSpeechMessage _SpeechMsg;
typedef std::shared_ptr<_SpeechMsg> _SpeechMsgPtr;

typedef CodedMessage<> _CodedMsg;
typedef std::shared_ptr<_CodedMsg> _CodedMsgPtr;

uint32_t num_color_ = 0;
uint32_t num_depth_ = 0;
uint32_t num_infrared_ = 0;
uint32_t num_audio_ = 0;
uint32_t num_bodies_ = 0;
uint32_t num_speech_ = 0;

class ColorImageReadTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_ColorImageMsgPtr> > _ColorImageOutputFifo;

    _ColorImageOutputFifo & output_fifo_;
    KinectDevice & kinect_device_;
    bool running_;

    ColorImageReadTask( _ColorImageOutputFifo & output_fifo, KinectDevice & kinect_device )
    :
        output_fifo_( output_fifo ),
        kinect_device_( kinect_device ),
        running_( true )
    {
        //
    }

    void run()
    {
        _ColorImageMsgPtr message_ptr;
        _ColorImageMsgPtr cropped_message_ptr;
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*16 && output_handle.waitOn()->size() >= 2*16 )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }
            }

            try
            {
//                std::cout << "pulling color image" << std::endl;
                kinect_device_.pullColorImage( message_ptr, "RGBA" );
                message_ptr->header_.num_channels_ = 3;

                // crop here

                if( !cropped_message_ptr ) cropped_message_ptr = std::make_shared<_ColorImageMsg>();

                uint16_t const crop_x = message_ptr->header_.width_ * 0.4f;
                uint16_t const crop_y = message_ptr->header_.height_ * 0.25f;
                uint16_t const crop_w = message_ptr->header_.width_ - ( 2.0f * crop_x );
                uint16_t const crop_h = message_ptr->header_.height_ - ( crop_y + 0.8f*crop_y );

                cropped_message_ptr->header_ = message_ptr->header_;
                cropped_message_ptr->header_.width_ = crop_w;
                cropped_message_ptr->header_.height_ = crop_h;
                cropped_message_ptr->header_.encoding_ = "rgb";

                cropped_message_ptr->payload_.allocate( cropped_message_ptr->header_.width_ * cropped_message_ptr->header_.height_ * cropped_message_ptr->header_.num_channels_ * cropped_message_ptr->header_.pixel_depth_ / 8 );

                auto & input_bytes = message_ptr->payload_.payload_;
                auto & output_bytes = cropped_message_ptr->payload_.payload_;

                for( size_t row = crop_y; row < crop_y + crop_h; ++row )
                {
                    for( size_t col = crop_x; col < crop_x + crop_w; ++col )
                    {
                        // only copy relevant data
                        size_t const input_idx = row * message_ptr->header_.width_ + col;
                        size_t const output_idx = ( row - crop_y ) * crop_w + ( col - crop_x );
                        // only copy RGB (we ignore A anyway)
                        std::memcpy( output_bytes + 3 * output_idx, input_bytes + 4 * input_idx, 3 );
                    }
                }

                output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ALL )->push_back( cropped_message_ptr );
            }
            catch( KinectException & e )
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                continue;
            }

        }
    }
};

class DepthImageReadTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_DepthImageMsgPtr> > _OutputFifo;

    _OutputFifo & output_fifo_;
    KinectDevice & kinect_device_;
    bool running_;

    DepthImageReadTask( _OutputFifo & output_fifo, KinectDevice & kinect_device )
    :
        output_fifo_( output_fifo ),
        kinect_device_( kinect_device ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*16 && output_handle.waitOn()->size() >= 2*16 )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }
            }


            try
            {
//                std::cout << "pulling depth image" << std::endl;
                _DepthImageMsgPtr message_ptr;
                kinect_device_.pullDepthImage( message_ptr );
                output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ALL )->push_back( message_ptr );
            }
            catch( KinectException & e )
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                continue;
            }

        }
    }
};

class InfraredImageReadTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_InfraredImageMsgPtr> > _OutputFifo;

    _OutputFifo & output_fifo_;
    KinectDevice & kinect_device_;
    bool running_;

    InfraredImageReadTask( _OutputFifo & output_fifo, KinectDevice & kinect_device )
    :
        output_fifo_( output_fifo ),
        kinect_device_( kinect_device ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*16 && output_handle.waitOn()->size() >= 2*16 )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }
            }


            try
            {
//                std::cout << "pulling infrared image" << std::endl;
                _InfraredImageMsgPtr message_ptr;
                kinect_device_.pullInfraredImage( message_ptr );
                output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ALL )->push_back( message_ptr );
            }
            catch( KinectException & e )
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                continue;
            }

        }
    }
};

class AudioReadTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_AudioMsgPtr> > _OutputFifo;

    _OutputFifo & output_fifo_;
    KinectDevice & kinect_device_;
    bool running_;

    AudioReadTask( _OutputFifo & output_fifo, KinectDevice & kinect_device )
    :
        output_fifo_( output_fifo ),
        kinect_device_( kinect_device ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*16 && output_handle.waitOn()->size() >= 2*16 )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }
            }

            auto message_ptr = std::make_shared<_AudioMsg>();

//            std::cout << "pulling audio" << std::endl;
            // get at least 8 frames of audio data
            std::vector<_AudioMsgPtr> frame_ptrs;
            while( message_ptr->header_.num_samples_ < 2048 )
            {
                try
                {
                    _AudioMsgPtr frame_ptr;
                    kinect_device_.pullAudio( frame_ptr );

                    message_ptr->payload_.size_ += frame_ptr->payload_.size_;
                    message_ptr->header_.num_samples_ += frame_ptr->header_.num_samples_;
                    message_ptr->beam_angle_ += frame_ptr->beam_angle_;
                    message_ptr->beam_angle_confidence_ += frame_ptr->beam_angle_confidence_;

                    frame_ptrs.push_back( frame_ptr );
                }
                catch( KinectException & e )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
                    continue;
                }
            }

            // combine all messages into one message

            message_ptr->header_.num_channels_ = frame_ptrs.front()->header_.num_channels_;
            message_ptr->header_.sample_depth_ = frame_ptrs.front()->header_.sample_depth_;
            message_ptr->header_.sample_rate_ = frame_ptrs.front()->header_.sample_rate_;
            message_ptr->header_.encoding_ = frame_ptrs.front()->header_.encoding_;

            // there are 256 samples in each frame; we need to normalize beam angle stuff on a per-frame basis
            message_ptr->beam_angle_ /= ( message_ptr->header_.num_samples_ / 256 );
            message_ptr->beam_angle_confidence_ /= ( message_ptr->header_.num_samples_ / 256 );

            message_ptr->payload_.allocate();

            uint32_t payload_offset = 0;
            for( auto frame_ptrs_it = frame_ptrs.cbegin(); frame_ptrs_it != frame_ptrs.cend(); ++frame_ptrs_it )
            {
                auto & frame_ptr = *frame_ptrs_it;

                std::memcpy( message_ptr->payload_.data_ + payload_offset, frame_ptr->payload_.data_, frame_ptr->payload_.size_ );

                payload_offset += frame_ptr->payload_.size_;
            }

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ALL )->push_back( message_ptr );
        }
    }
};

class BodiesReadTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_BodiesMsgPtr> > _OutputFifo;

    _OutputFifo & output_fifo_;
    KinectDevice & kinect_device_;
    bool running_;

    BodiesReadTask( _OutputFifo & output_fifo, KinectDevice & kinect_device )
    :
        output_fifo_( output_fifo ),
        kinect_device_( kinect_device ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*16 && output_handle.waitOn()->size() >= 2*16 )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }
            }


            try
            {
//                std::cout << "pulling bodies" << std::endl;
                _BodiesMsgPtr message_ptr;
                kinect_device_.pullBodies( message_ptr );
                output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ALL )->push_back( message_ptr );
            }
            catch( KinectException & e )
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                continue;
            }

        }
    }
};

class SpeechReadTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_SpeechMsgPtr> > _OutputFifo;

    _OutputFifo & output_fifo_;
    KinectDevice & kinect_device_;
    bool running_;

    SpeechReadTask( _OutputFifo & output_fifo, KinectDevice & kinect_device )
    :
        output_fifo_( output_fifo ),
        kinect_device_( kinect_device ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*16 && output_handle.waitOn()->size() >= 2*16 )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }
            }


            try
            {
//                std::cout << "pulling bodies" << std::endl;
                _SpeechMsgPtr message_ptr;
                kinect_device_.pullSpeech( message_ptr );
                if( message_ptr->size() > 0 ) output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ALL )->push_back( message_ptr );
            }
            catch( KinectException & e )
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                continue;
            }

        }
    }
};

class ColorImageCompressTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_ColorImageMsgPtr> > _InputFifo;
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _OutputFifo;
    typedef MessageCoder<BinaryCodec<> > _MessageCoder;

    _InputFifo & input_fifo_;
    _OutputFifo & output_fifo_;
    _MessageCoder message_coder_;
    bool running_;

    ColorImageCompressTask( _InputFifo & input_fifo, _OutputFifo & output_fifo, _MessageCoder & message_coder )
    :
        input_fifo_( input_fifo ),
        output_fifo_( output_fifo ),
        message_coder_( message_coder ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*32 && output_handle.waitOn()->size() >= 2*32 ) continue;
            }

            _ColorImageMsgPtr raw_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() ) continue;

                raw_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

//            std::cout << "compressing color image" << std::endl;

            raw_message_ptr->compression_level_ = 1;

            _CodedMsgPtr output_message_ptr = std::make_shared<_CodedMsg>( message_coder_.encode( *raw_message_ptr ) );

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE )->push_back( output_message_ptr );
        }

//        std::cout << "!! ColorImageCompressTask done" << std::endl;
    }
};

class DepthImageCompressTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_DepthImageMsgPtr> > _InputFifo;
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _OutputFifo;
    typedef MessageCoder<BinaryCodec<> > _MessageCoder;

    _InputFifo & input_fifo_;
    _OutputFifo & output_fifo_;
    _MessageCoder message_coder_;
    bool running_;

    DepthImageCompressTask( _InputFifo & input_fifo, _OutputFifo & output_fifo, _MessageCoder & message_coder )
    :
        input_fifo_( input_fifo ),
        output_fifo_( output_fifo ),
        message_coder_( message_coder ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*32 && output_handle.waitOn()->size() >= 2*32 ) continue;
            }

            _DepthImageMsgPtr raw_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() ) continue;

                raw_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

//            std::cout << "compressing depth image" << std::endl;

            raw_message_ptr->compression_level_ = 1;

            _CodedMsgPtr output_message_ptr = std::make_shared<_CodedMsg>( message_coder_.encode( *raw_message_ptr ) );

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE )->push_back( output_message_ptr );
        }

//        std::cout << "!! DepthImageCompressTask done" << std::endl;
    }
};

class InfraredImageCompressTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_InfraredImageMsgPtr> > _InputFifo;
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _OutputFifo;
    typedef MessageCoder<BinaryCodec<> > _MessageCoder;

    _InputFifo & input_fifo_;
    _OutputFifo & output_fifo_;
    _MessageCoder message_coder_;
    bool running_;

    InfraredImageCompressTask( _InputFifo & input_fifo, _OutputFifo & output_fifo, _MessageCoder & message_coder )
    :
        input_fifo_( input_fifo ),
        output_fifo_( output_fifo ),
        message_coder_( message_coder ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*32 && output_handle.waitOn()->size() >= 2*32 ) continue;
            }

            _InfraredImageMsgPtr raw_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() ) continue;

                raw_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

//            std::cout << "compressing infrared image" << std::endl;

            raw_message_ptr->compression_level_ = 1;

            _CodedMsgPtr output_message_ptr = std::make_shared<_CodedMsg>( message_coder_.encode( *raw_message_ptr ) );

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE )->push_back( output_message_ptr );
        }

//        std::cout << "!! InfraredImageCompressTask done" << std::endl;
    }
};

class AudioCompressTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_AudioMsgPtr> > _InputFifo;
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _OutputFifo;
    typedef MessageCoder<GZipCodec<> > _MessageCoder;

    _InputFifo & input_fifo_;
    _OutputFifo & output_fifo_;
    _MessageCoder message_coder_;
    bool running_;

    AudioCompressTask( _InputFifo & input_fifo, _OutputFifo & output_fifo, _MessageCoder & message_coder )
    :
        input_fifo_( input_fifo ),
        output_fifo_( output_fifo ),
        message_coder_( message_coder ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*32 && output_handle.waitOn()->size() >= 2*32 ) continue;
            }

            _AudioMsgPtr raw_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() ) continue;

                raw_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

//            std::cout << "compressing audio" << std::endl;

            _CodedMsgPtr output_message_ptr = std::make_shared<_CodedMsg>( message_coder_.encode( *raw_message_ptr ) );

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE )->push_back( output_message_ptr );
        }

//        std::cout << "!! AudioCompressTask done" << std::endl;
    }
};

class BodiesCompressTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_BodiesMsgPtr> > _InputFifo;
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _OutputFifo;
    typedef MessageCoder<BinaryCodec<> > _MessageCoder;

    _InputFifo & input_fifo_;
    _OutputFifo & output_fifo_;
    _MessageCoder message_coder_;
    bool running_;

    BodiesCompressTask( _InputFifo & input_fifo, _OutputFifo & output_fifo, _MessageCoder & message_coder )
    :
        input_fifo_( input_fifo ),
        output_fifo_( output_fifo ),
        message_coder_( message_coder ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*32 && output_handle.waitOn()->size() >= 2*32 ) continue;
            }

            _BodiesMsgPtr raw_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() ) continue;

                raw_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

//            std::cout << "compressing bodies" << std::endl;

            _CodedMsgPtr output_message_ptr = std::make_shared<_CodedMsg>( message_coder_.encode( *raw_message_ptr ) );

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE )->push_back( output_message_ptr );
        }

//        std::cout << "!! BodiesCompressTask done" << std::endl;
    }
};

class SpeechCompressTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_SpeechMsgPtr> > _InputFifo;
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _OutputFifo;
    typedef MessageCoder<BinaryCodec<> > _MessageCoder;

    _InputFifo & input_fifo_;
    _OutputFifo & output_fifo_;
    _MessageCoder message_coder_;
    bool running_;

    SpeechCompressTask( _InputFifo & input_fifo, _OutputFifo & output_fifo, _MessageCoder & message_coder )
    :
        input_fifo_( input_fifo ),
        output_fifo_( output_fifo ),
        message_coder_( message_coder ),
        running_( true )
    {
        //
    }

    void run()
    {
        while( running_ )
        {
            // if the output fifo is too full, wait for consumers to pop items off
            {
                auto output_handle = output_fifo_.getHandle();

                if( output_handle->size() >= 2*32 && output_handle.waitOn()->size() >= 2*32 ) continue;
            }

            _SpeechMsgPtr raw_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() ) continue;

                raw_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

//            std::cout << "compressing bodies" << std::endl;

            _CodedMsgPtr output_message_ptr = std::make_shared<_CodedMsg>( message_coder_.encode( *raw_message_ptr ) );

            output_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE )->push_back( output_message_ptr );
        }

//        std::cout << "!! SpeechCompressTask done" << std::endl;
    }
};

class WriteTask : public Poco::Runnable
{
public:
    typedef atomics::Wrapper<std::deque<_CodedMsgPtr> > _InputFifo;

    _InputFifo & input_fifo_;
    std::ofstream & output_stream_;
    bool running_;

    WriteTask( _InputFifo & input_fifo, std::ofstream & output_stream )
    :
        input_fifo_( input_fifo ),
        output_stream_( output_stream ),
        running_( true )
    {
        //
    }

    void run()
    {
        Poco::Timestamp timer;

        while( running_ )
        {
            _CodedMsgPtr compressed_message_ptr;
            // if the input fifo is empty, wait for producers to push items on
            {
                auto input_handle = input_fifo_.getHandle( atomics::HandleBase::NotifyType::DELAY_NOTIFY_ONE );

                if( input_handle->empty() && input_handle.waitOn()->empty() )
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / 35 ) );
                    continue;
                }

                compressed_message_ptr = input_handle->front();
                input_handle->pop_front();
            }

            if( compressed_message_ptr->header_.payload_id_ == _ColorImageMsg::ID() ) num_color_ ++;
            else if( compressed_message_ptr->header_.payload_id_ == _DepthImageMsg::ID() ) num_depth_ ++;
            else if( compressed_message_ptr->header_.payload_id_ == _InfraredImageMsg::ID() ) num_infrared_ ++;
            else if( compressed_message_ptr->header_.payload_id_ == _AudioMsg::ID() ) num_audio_ ++;
            else if( compressed_message_ptr->header_.payload_id_ == _BodiesMsg::ID() ) num_bodies_ ++;
            else if( compressed_message_ptr->header_.payload_id_ == _SpeechMsg::ID() ) num_speech_ ++;

            compressed_message_ptr->pack( output_stream_ );
        }
    }
};

bool running_ = true;

// ctrl-c detection for windows
BOOL WINAPI sigkillHandler( DWORD signal )
{
    if( signal == CTRL_C_EVENT )
    {
        running_ = false;
    }

    return TRUE;
}

int main()
{
    // ctrl-c detection for windows
    SetConsoleCtrlHandler( sigkillHandler, TRUE );

    KinectDevice kinect_device;

    std::cout << "Waiting for Kinect to become ready" << std::endl;
    while( true )
    {
        try
        {
            kinect_device.initialize( true, true, true, true, true );
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );
            continue;
        }
    }

    std::stringstream ss;

    ss << "D:/chla/data/kinectv2-" << std::time( 0 ) << ".pak";

    std::cout << "logging to: " << ss.str() << std::endl;
    std::ofstream output_stream( ss.str(), std::ios::binary );

    ColorImageCompressTask::_MessageCoder color_image_message_coder;
    DepthImageCompressTask::_MessageCoder depth_image_message_coder;
    InfraredImageCompressTask::_MessageCoder infrared_image_message_coder;
    AudioCompressTask::_MessageCoder audio_message_coder( 1 );
    BodiesCompressTask::_MessageCoder bodies_message_coder;
    SpeechCompressTask::_MessageCoder speech_message_coder;

    // set threadpool sizes
    Poco::ThreadPool read_pool( 6, 6 );
    Poco::ThreadPool compress_pool( 8 + 2 + 2 + 1 + 1 + 1, 8 + 2 + 2 + 1 + 1 + 1 );
    Poco::ThreadPool write_pool( 1, 1 );

    // declare inter-thread message passing FIFOs
    atomics::Wrapper<std::deque<_ColorImageMsgPtr> > color_image_read_fifo;
    atomics::Wrapper<std::deque<_DepthImageMsgPtr> > depth_image_read_fifo;
    atomics::Wrapper<std::deque<_InfraredImageMsgPtr> > infrared_image_read_fifo;
    atomics::Wrapper<std::deque<_AudioMsgPtr> > audio_read_fifo;
    atomics::Wrapper<std::deque<_BodiesMsgPtr> > bodies_read_fifo;
    atomics::Wrapper<std::deque<_SpeechMsgPtr> > speech_read_fifo;

    atomics::Wrapper<std::deque<_CodedMsgPtr> > compress_fifo;

    // declare read tasks
    ColorImageReadTask color_image_read_task( color_image_read_fifo, kinect_device );
    DepthImageReadTask depth_image_read_task( depth_image_read_fifo, kinect_device );
    InfraredImageReadTask infrared_image_read_task( infrared_image_read_fifo, kinect_device );
    AudioReadTask audio_read_task( audio_read_fifo, kinect_device );
    BodiesReadTask bodies_read_task( bodies_read_fifo, kinect_device );
    SpeechReadTask speech_read_task( speech_read_fifo, kinect_device );

    // declare compress tasks
    ColorImageCompressTask color_image_compress_task( color_image_read_fifo, compress_fifo, color_image_message_coder );
    DepthImageCompressTask depth_image_compress_task( depth_image_read_fifo, compress_fifo, depth_image_message_coder );
    InfraredImageCompressTask infrared_image_compress_task( infrared_image_read_fifo, compress_fifo, infrared_image_message_coder );
    AudioCompressTask audio_compress_task( audio_read_fifo, compress_fifo, audio_message_coder );
    BodiesCompressTask bodies_compress_task( bodies_read_fifo, compress_fifo, bodies_message_coder );
    SpeechCompressTask speech_compress_task( speech_read_fifo, compress_fifo, speech_message_coder );

    // declare output tasks
    WriteTask write_task( compress_fifo, output_stream );

    std::cout << "starting worker threads" << std::endl;

    // start pipeline in reverse, starting with outputs
    while( write_pool.available() ) write_pool.start( write_task );

    for( size_t i = 0; i < 8; ++i ) compress_pool.start( color_image_compress_task );
    for( size_t i = 0; i < 2; ++i ) compress_pool.start( depth_image_compress_task );
    for( size_t i = 0; i < 2; ++i ) compress_pool.start( infrared_image_compress_task );
    for( size_t i = 0; i < 1; ++i ) compress_pool.start( audio_compress_task );
    for( size_t i = 0; i < 1; ++i ) compress_pool.start( bodies_compress_task );
    for( size_t i = 0; i < 1; ++i ) compress_pool.start( speech_compress_task );

    read_pool.start( color_image_read_task );
    read_pool.start( depth_image_read_task );
    read_pool.start( infrared_image_read_task );
    read_pool.start( audio_read_task );
    read_pool.start( bodies_read_task );
    read_pool.start( speech_read_task );

    // use the main thread to produce status updates while program is running
    std::streamsize last_size = 0;
    while( running_ )
    {
        std::streamsize current_size( output_stream.tellp() );
        std::cout << "Output MBytes: " << current_size / 1000000 << " (" << static_cast<int>( ( 1000.0 / 500.0 ) * ( current_size - last_size ) / 1000000 ) << " MB/sec)" << std::endl;
        std::cout << num_color_ << " | " << num_depth_ << " | " << num_infrared_ << " | " << num_audio_ << " | " << num_bodies_ << " | " << num_speech_ << std::endl;
        last_size = current_size;
        std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
    }

    // on shutdown, stop tasks in the oppisite order they were started, so from inputs to outputs
    std::cout << "stopping worker threads" << std::endl;

    // mark all read tasks as stopped; we assume they are single-threaded, non-blocking, and will check their own states reasonably often
    color_image_read_task.running_ = false;
    depth_image_read_task.running_ = false;
    infrared_image_read_task.running_ = false;
    audio_read_task.running_ = false;
    bodies_read_task.running_ = false;
    speech_read_task.running_ = false;
    read_pool.joinAll();

    std::cout << "read threads stopped" << std::endl;

    // for each compression task, we assume they are multi-threaded and are potentially blocking on their input FIFOs
    // we also assume that they are not deadlocked waiting on their output FIFOs
    // at this point, all read threads are stopped, so we we ping the input FIFOs to wake up the compression threads and force them to empty out the input FIFOs
    while( !color_image_read_fifo->empty() )
    {
        color_image_read_fifo.getCondition().notify_one();
    }
    std::cout << "compress color image task done" << std::endl;
    while( !depth_image_read_fifo->empty() )
    {
        depth_image_read_fifo.getCondition().notify_one();
    }
    std::cout << "compress depth image task done" << std::endl;
    while( !infrared_image_read_fifo->empty() )
    {
        infrared_image_read_fifo.getCondition().notify_one();
    }
    std::cout << "compress infrared image task done" << std::endl;
    while( !audio_read_fifo->empty() )
    {
        audio_read_fifo.getCondition().notify_one();
    }
    std::cout << "compress audio task done" << std::endl;
    while( !bodies_read_fifo->empty() )
    {
        bodies_read_fifo.getCondition().notify_one();
    }
    std::cout << "compress bodies task done" << std::endl;
    while( !speech_read_fifo->empty() )
    {
        speech_read_fifo.getCondition().notify_one();
    }
    std::cout << "compress speech task done" << std::endl;

    std::cout << "compression threads stopped" << std::endl;

    // at this point, all compression threads should have empty inputs, so we can shut them down
    color_image_compress_task.running_ = false;
    depth_image_compress_task.running_ = false;
    infrared_image_compress_task.running_ = false;
    audio_compress_task.running_ = false;
    bodies_compress_task.running_ = false;
    speech_compress_task.running_ = false;

    color_image_read_fifo.getCondition().notify_all();
    depth_image_read_fifo.getCondition().notify_all();
    infrared_image_read_fifo.getCondition().notify_all();
    audio_read_fifo.getCondition().notify_all();
    bodies_read_fifo.getCondition().notify_all();
    speech_read_fifo.getCondition().notify_all();

    // wait for compression threads to stop
    compress_pool.joinAll();

    std::cout << "compress threads stopped" << std::endl;

    // ping compression inputs until they're empty, then stop the write threads
    while( !compress_fifo->empty() ) compress_fifo.getCondition().notify_one();
    write_task.running_ = false;
    compress_fifo.getCondition().notify_all();
    write_pool.joinAll();

    // close output stream
    std::cout << "closing output stream" << std::endl;

    output_stream.flush();

    std::cout << "final log size: " << output_stream.tellp() / 1000000 << std::endl;
    output_stream.close();

    // give users a chance to read the statistics before exiting
    std::cout << "terminating in 3 seconds..." << std::endl;

    std::this_thread::sleep_for( std::chrono::seconds( 3 ) );

    return 0;
}
