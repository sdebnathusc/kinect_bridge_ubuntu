#ifndef _MESSAGES_BINARYMESSAGE_H_
#define _MESSAGES_BINARYMESSAGE_H_

#include <messages/serializable_message.h>
#include <cstring>

template<class __Allocator = std::allocator<char> >
class BinaryMessage : public SerializableMessageInterface<uint32_t, char *>
{
public:
    typedef SerializableMessageInterface<uint32_t, char *> _Message;

    uint32_t & size_;
    char *& data_;
    bool owns_;

    __Allocator allocator_;

/*
    BinaryMessage()
    :
        _Message( std::forward<uint32_t>( 0 ), std::forward<char *>( NULL ) ),
        size_( this->header_ ),
        data_( this->payload_ ),
        owns_( false )

    {
        std::cout << name() << " default constructor: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
    }
*/

/*
    BinaryMessage( char * data, uint32_t size )
    :
        _Message( std::forward<uint32_t>( size ), std::forward<char *>( data ) ),
        size_( this->header_ ),
        data_( this->payload_ )
    {
        std::cout << name() << " normal constructor" << std::endl;
    }
*/

    BinaryMessage( char const * data = NULL, uint32_t size = 0 )
    :
        _Message( std::forward<uint32_t>( size ), const_cast<char *>( std::forward<char const *>( data ) ) ),
        size_( this->header_ ),
        data_( this->payload_ ),
        owns_( false )
    {
//        std::cout << name() << " normal, const data constructor: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
    }

    BinaryMessage( BinaryMessage<__Allocator> const & other )
    :
        _Message( std::forward<uint32_t>( other.size_ ), std::forward<char *>( allocator_.allocate( other.size_ ) ) ),
        size_( this->header_ ),
        data_( this->payload_ ),
        owns_( true )
    {
        std::memcpy( data_, other.data_, size_ );
//        std::cout << name() << " copy constructor, same alloc: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
//        std::cout << "allocated: " << static_cast<void *>( data_ ) << " (" << size_ << ")" << std::endl;
    }

    template<class __OtherAllocator>
    BinaryMessage( BinaryMessage<__OtherAllocator> const & other )
    :
        _Message( std::forward<uint32_t>( other.size_ ), std::forward<char *>( allocator_.allocate( other.size_ ) ) ),
        size_( this->header_ ),
        data_( this->payload_ ),
        owns_( true )
    {
        std::memcpy( data_, other.data_, size_ );
//        std::cout << name() << " copy constructor: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
        std::cout << "allocated: " << static_cast<void *>( data_ ) << " (" << size_ << ")" << std::endl;
    }

    BinaryMessage( BinaryMessage<__Allocator> && other )
    :
        _Message( std::forward<uint32_t>( other.size_ ), std::forward<char *>( other.data_ ) ),
        size_( this->header_ ),
        data_( this->payload_ ),
        owns_( other.owns_ )
    {
        other.size_ = 0;
        other.data_ = NULL;
        // if we just took ownership
        if( owns_ ) other.owns_ = false;

//        std::cout << name() << " move constructor, same alloc: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
    }

    template<class __OtherAllocator>
    BinaryMessage( BinaryMessage<__OtherAllocator> && other )
    :
        _Message( std::forward<uint32_t>( other.size_ ), std::forward<char *>( other.data_ ) ),
        size_( this->header_ ),
        data_( this->payload_ ),
        owns_( other.owns_ )
    {
        other.size_ = 0;
        other.data_ = NULL;
        // if we just took ownership
        if( owns_ ) other.owns_ = false;

//        std::cout << name() << " move constructor: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
    }

    BinaryMessage<__Allocator> & operator=( BinaryMessage<__Allocator> const & other )
    {
        if( this != &other )
        {
//            std::cout << name() << " copy operator: " << static_cast<void *>( other.data_ ) << ", " << other.size_ << std::endl;
            deallocate();
            size_ = other.size_;
            data_ = allocator_.allocate( size_ );
            owns_ = true;
            std::memcpy( data_, other.data_, size_ );
        }
        else
        {
//            std::cout << name() << " self assignment operator: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
        }

        return *this;
    }

    void deallocate()
    {
        if( data_ && owns_ )
        {
//            std::cout << "deallocating memory" << std::endl;
            allocator_.deallocate( data_, size_ );
        }
    }

    ~BinaryMessage()
    {
//        std::cout << "BinaryMessage destructor: " << static_cast<void *>( data_ ) << ", " << size_ << std::endl;
        deallocate();
    }

    template<class __Archive>
    void packHeader( __Archive & archive ) const
    {
//        std::cout << "packing BinaryMessage header; size: " << size_ << std::endl;
        archive << size_;
    }

    template<class __Archive>
    void packPayload( __Archive & archive ) const
    {
//        std::cout << "packing BinaryMessage payload; size: " << size_ << std::endl;
        archive.write( data_, size_ );
    }

    template<class __Archive>
    void unpackHeader( __Archive & archive )
    {
//        std::cout << "unpacking BinaryMessage header" << std::endl;
        archive >> size_;
//        std::cout << "unpacked BinaryMessage header; size: " << size_ << std::endl;
    }

    void allocate( size_t size )
    {
//        std::cout << "allocating memory prior to unpacking" << std::endl;
        if( data_ && size_ >= size )
        {
//            std::cout << "memory already allocated" << std::endl;
            return;
        }
        data_ = reinterpret_cast<char *>( allocator_.allocate( size ) );
        size_ = size;
        owns_ = true;
//        std::cout << "allocated: " << static_cast<void *>( data_ ) << " (" << size_ << ")" << std::endl;
    }

    void allocate()
    {
        allocate( size_ );
    }

    template<class __Archive>
    void unpackPayload( __Archive & archive )
    {
//        std::cout << "unpacking BinaryMessage payload; size: " << size_ << std::endl;
        if( !data_ )
        {
            allocate();
        }
        archive.read( data_, size_ );
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        _Message::pack( *this, archive );
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        _Message::unpack( *this, archive );
    }

    DECLARE_MESSAGE_INFO( BinaryMessage )
};

#endif // _MESSAGES_BINARYMESSAGE_H_
