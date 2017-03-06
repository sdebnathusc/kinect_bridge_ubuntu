#ifndef _KINECTCOMMON_D2DRENDERER_H_
#define _KINECTCOMMON_D2DRENDERER_H_

#include <kinect_common/direct_2d.h>
#include <messages/image_message.h>

class D2DRenderer
{
public:
    typedef ReleasableWrapper<ID2D1Factory> D2D1Factory;
    typedef ReleasableWrapper<ID2D1HwndRenderTarget> D2D1HwndRenderTarget;
    typedef ReleasableWrapper<ID2D1Bitmap> D2D1Bitmap;

    D2D1Factory d2d_factory_;
    D2D1HwndRenderTarget render_target_;
    D2D1Bitmap bitmap_;

    HWND video_view_handle_;

    uint16_t width_;
    uint16_t height_;
    std::string encoding_;
    uint32_t stride_;
    D2D1::PixelFormat pixel_format_;

    D2DRenderer( HWND video_view_handle )
    :
        video_view_handle_( video_view_handle )
    {
        D2D1CreateFactory( D2D1_FACTORY_TYPE_SINGLE_THREADED, &d2d_factory_.get() );
    }

    void initialize( uint16_t width, uint16_t height, uint8_t num_channels, uint8_t pixel_depth, std::string encoding )
    {
        width_ = width;
        height_ = height;
        encoding_ = encoding;
        stride_ = width * num_channels * pixel_depth / 8;

        // select pixel format from encoding, etc
        pixel_format_ = D2D1::PixelFormat( DXGI_FORMAT_R8G8B8A8_UNORM, D2D1_ALPHA_MODE_IGNORE );

        d2d_factory_->AddRef();
    }

    void allocateResources()
    {
        if( render_target_.get() ) return;

        D2D1_RENDER_TARGET_PROPERTIES render_properties = D2D1::RenderTargetProperties();
        render_properties.pixelFormat = pixel_format_;
        render_properties.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        D2D1_SIZE_U size = D2D1::SizeU( width, height );

        // Create a hWnd render target, in order to render to the window set in initialize
        d2d_factory_->CreateHwndRenderTarget( render_properties, D2D1::HwndRenderTargetProperties( video_view_handle_, size ), &render_target_.get() );

        render_target_->CreateBitmap( size, pixel_format_, &bitmap_.get() );
    }

    void deallocateResources()
    {
        render_target_.release();
        bitmap_.release();
    }

    // this should be a BinaryMessage<>
    template<class __Serializable>
    void push( __Serializable & serializable )
    {
        auto & size = serializable.size_;
        auto & data = serializable.data_;

        allocateResources();

        // Copy the image that was passed in into the direct2d bitmap
        bitmap_->CopyFromMemory( NULL, payload.data_, stride_ );

        render_target_->BeginDraw();

        // Draw the bitmap stretched to the size of the window
        render_target_->DrawBitmap( bitmap_.get() );

        // Device lost, need to recreate the render target
        // We'll dispose it now and retry drawing
        if( render_target_->EndDraw() == D2DERR_RECREATE_TARGET )
        {
            deallocateResources();
        }
    }

    template<class __Payload>
    void push( ImageMessage<__Payload> & image_message )
    {
        auto & header = image_message.header_;
        auto & payload = image_message.payload_;

        initialize( header.width_, header.height_, header.num_channels_, header.pixel_depth_, header.encoding_ );

        push( payload );
    }
};

#endif // _KINECTCOMMON_D2DRENDERER_H_
