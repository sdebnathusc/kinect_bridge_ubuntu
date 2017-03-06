#include <iostream>
#include <fstream>
#include <thread>

#include <kinect_common/kinect_device.h>
#include <messages/png_image_message.h>

// will accept any message type derived from ImageMessage
// we use PNGImageMessage to allow for PNG compression when packing
// using ImageMessage<> instead would not allow for compression without copying/casting
typedef KinectColorImageMessage<PNGImageMessage<> > _ColorImageMsg;

int main()
{
    std::ofstream output_stream1( "test_kinect_RGB.bin", std::ios::binary );
    std::ofstream output_stream2( "test_kinect_RGB.png", std::ios::binary );
    std::ofstream output_stream3( "test_kinect_RGB.pak", std::ios::binary );

    KinectDevice kinect_device;
    while( true )
    {
        try
        {
            kinect_device.initialize( true, false, false, false );
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );
            continue;
        }
    }

    std::shared_ptr<_ColorImageMsg> message_ptr;
    std::shared_ptr<_ColorImageMsg> cropped_message_ptr;

    while( true )
    {
        try
        {
            kinect_device.pullColorImage( message_ptr, "RGB" );
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
                    std::memcpy( output_bytes + 3 * output_idx, input_bytes + 3 * input_idx, 3 );
                }
            }

            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
            continue;
        }
    }

    std::cout << "packing raw image" << std::endl;

    if( !cropped_message_ptr ) std::cout << "output message not defined" << std::endl;

    // first, write raw image (3-channel RGB)
    cropped_message_ptr->packAs<ImageMessage<> >( output_stream1 );

    std::cout << "packing png image" << std::endl;

    // now, encode as png
    cropped_message_ptr->packAs<PNGImageMessage<> >( output_stream2 );

    std::cout << "packing kinect image" << std::endl;

    // save entire message including timestamp
    cropped_message_ptr->pack( output_stream3 );

    output_stream1.close();
    output_stream2.close();
    output_stream3.close();

    return 0;
}
