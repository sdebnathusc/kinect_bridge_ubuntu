#ifndef _ATOMICS_MEMBLOCK_H_
#define _ATOMICS_MEMBLOCK_H_

#include <atomics/print.h>
#include <Poco/BinaryReader.h>
#include <Poco/BinaryWriter.h>

namespace atomics
{

template<class __Data1, class __Data2, class __Enable = void>
struct BasicMemBlockConverter;

// specialization for converting to single-byte data type
template<class __Data1, class __Data2>
struct BasicMemBlockConverter<__Data1, __Data2, typename std::enable_if<(sizeof(__Data2)==1)>::type>
{
    // convert size in __Data1 units to __Data2 units
    inline static size_t size( size_t size )
    {
        return size * sizeof( __Data1 );
    }

    // convert data in __Data1 units to __Data2 units
    inline static __Data2 * data( __Data1 * data )
    {
        return reinterpret_cast<__Data2 *>( data );
    }
};

// specialization for converting to multi-byte data type
template<class __Data1, class __Data2>
struct BasicMemBlockConverter<__Data1, __Data2, typename std::enable_if<(sizeof(__Data2)>1)>::type>
{
    // convert size in __Data1 units to __Data2 units
    inline static size_t size( size_t size )
    {
        return size * sizeof( __Data1 ) / sizeof( __Data2 );
    }

    // convert data in __Data1 units to __Data2 units
    inline static __Data2 * data( __Data1 * data )
    {
        return reinterpret_cast<__Data2 *>( data );
    }
};

template<class __Data = char>
class BasicMemBlock
{
public:
    typedef BasicMemBlock<__Data> _BasicMemBlock;
    typedef __Data _Data;

    __Data * data_;
    size_t size_;

    // move, convert
    template<class __OtherData>
    BasicMemBlock( BasicMemBlock<__OtherData> && other )
    :
        data_( BasicMemBlockConverter<__OtherData, __Data>::data( other.data_ ) ),
        size_( BasicMemBlockConverter<__OtherData, __Data>::size( other.size_ ) )
    {
//        atomics_print( "BasicMemBlock move, convert" << std::endl );
        if( reinterpret_cast<_BasicMemBlock*>( &other ) != this )
        {
            other.data_ = NULL;
            other.size_ = 0;
        }
    }

    // move
    BasicMemBlock( _BasicMemBlock && other )
    :
        data_( other.data_ ),
        size_( other.size_ )
    {
//        atomics_print( "BasicMemBlock move" << std::endl );
        if( &other != this )
        {
            other.data_ = NULL;
            other.size_ = 0;
        }
    }

    // copy, convert
    template<class __OtherData>
    BasicMemBlock( BasicMemBlock<__OtherData> const & other )
    :
        data_( BasicMemBlockConverter<__OtherData, __Data>::data( other.data_ ) ),
        size_( BasicMemBlockConverter<__OtherData, __Data>::size( other.size_ ) )
    {
        //
//        atomics_print( "BasicMemBlock copy, convert" << std::endl );
    }

    // copy
    BasicMemBlock( _BasicMemBlock const & other )
    :
        data_( other.data_ ),
        size_( other.size_ )
    {
//        atomics_print( "BasicMemBlock copy" << std::endl );
    }

    // default
    BasicMemBlock( __Data * data = NULL, size_t size = 0 )
    :
        data_( data ),
        size_( size )
    {
//        atomics_print( "BasicMemBlock default" << std::endl );
    }

    __Data * begin()
    {
        return data_;
    }

    __Data * end()
    {
        return data_ + size_;
    }

    __Data const * begin() const
    {
        return data_;
    }

    __Data const * end() const
    {
        return data_ + size_;
    }

    template<class __OtherData>
    __OtherData * begin()
    {
        return dataConvert<__OtherData>( begin() );
    }

    template<class __OtherData>
    __OtherData * end()
    {
        return dataConvert<__OtherData>( end() );
    }

    template<class __OtherData>
    __OtherData const * begin() const
    {
        return dataConvert<__OtherData const>( begin() );
    }

    template<class __OtherData>
    __OtherData const * end() const
    {
        return dataConvert<__OtherData const>( end() );
    }

    size_t size() const
    {
        return size_;
    }

    template<class __OtherData>
    size_t size() const
    {
        return sizeConvert<__OtherData>( size() );
    }

    size_t sizeBytes() const
    {
//        return size_ * sizeof( __Data );
        return size<char>();
    }

    void store( __Data * data, size_t size )
    {
        data_ = data;
        size_ = size;
    }

    void store( __Data * data )
    {
        store( data, size_ );
    }

    __Data & at( size_t const & index )
    {
        return *(begin() + index);
    }

    __Data const & at( size_t const & index ) const
    {
        return *(begin() + index);
    }

    __Data & operator[]( size_t const & index )
    {
        return at( index );
    }

    __Data const & operator[]( size_t const & index ) const
    {
        return at( index );
    }

    // convert size in our units to __OtherData units
    template<class __OtherData>
    static size_t sizeConvert( size_t size )
    {
        return BasicMemBlockConverter<__Data, __OtherData>::size( size );
    }

    // convert data in our units to __OtherData units
    template<class __OtherData>
    static __OtherData * dataConvert( __Data * data )
    {
        return BasicMemBlockConverter<__Data, __OtherData>::data( data );
    }
};

template<class __Data, class __Allocator>
class BasicAllocated : public BasicMemBlock<__Data>
{
public:
    typedef BasicAllocated<__Data, __Allocator> _BasicAllocated;
    typedef BasicMemBlock<__Data> _BasicMemBlock;

    typedef __Data _Data;
    typedef __Allocator _Allocator;
    typedef typename __Allocator::value_type _AllocatorData;

    __Allocator allocator_;

    bool owns_;

    // move, convert
    template<class __OtherData, class __OtherAllocator>
    BasicAllocated( BasicAllocated<__OtherData, __OtherAllocator> && other )
    :
        _BasicMemBlock( other ),
        allocator_( std::move( other.allocator_ ) ),
        owns_( true )
    {
        other.owns_ = false;

//        atomics_print( "BasicAllocated move, convert" << std::endl );
        allocate();
    }

    // move
    BasicAllocated( _BasicAllocated && other )
    :
        _BasicMemBlock( other ),
        allocator_( std::move( other.allocator_ ) ),
        owns_( true )
    {
        other.owns_ = false;

//        atomics_print( "BasicAllocated move" << std::endl );
        allocate();
    }

    // copy, convert
    template<class __OtherData, class __OtherAllocator>
    BasicAllocated( BasicAllocated<__OtherData, __OtherAllocator> const & other )
    :
        _BasicMemBlock( NULL, BasicMemBlockConverter<__OtherData, __Data>::size( other.size() ) ),
        allocator_( other.allocator_ ),
        owns_( true )
    {
//        atomics_print( "BasicAllocated copy, convert" << std::endl );
        allocate();
    }

    // copy
    BasicAllocated( _BasicAllocated const & other )
    :
        _BasicMemBlock( NULL, other.size() ),
        allocator_( other.allocator_ ),
        owns_( true )
    {
//        atomics_print( "BasicAllocated copy" << std::endl );
        allocate();
    }

    // default
    BasicAllocated( size_t size = 0 )
    :
        _BasicMemBlock( NULL, size ),
        allocator_( __Allocator() ),
        owns_( true )
    {
//        atomics_print( "BasicAllocated default" << std::endl );
        allocate();
    }

    // convert, forward allocator
    template<template<class, class...> class __OtherAllocator, class __OtherAllocatorData, class... __Args>
    BasicAllocated( size_t size, __OtherAllocator<__OtherAllocatorData, __Args...> && allocator )
    :
//        _BasicMemBlock( NULL, BasicMemBlockConverter<__OtherAllocatorData, _AllocatorData>::size( size ) ),
        _BasicMemBlock( BasicMemBlock<__OtherAllocatorData>( NULL, size ) ),
        allocator_( std::forward<__OtherAllocator<__OtherAllocatorData, __Args...> >( allocator ) ),
        owns_( true )
    {
//        atomics_print( "BasicAllocated convert, forward allocator" << std::endl );
        allocate();
    }

    // convert, reference allocator
    template<template<class, class...> class __OtherAllocator, class __OtherAllocatorData, class... __Args>
    BasicAllocated( size_t size, __OtherAllocator<__OtherAllocatorData, __Args...> & allocator )
    :
        _BasicMemBlock( NULL, BasicMemBlockConverter<__OtherAllocatorData, _AllocatorData>::size( size ) ),
        allocator_( allocator ),
        owns_( true )
    {
//        atomics_print( "BasicAllocated convert, reference allocator" << std::endl );
        allocate();
    }

    // defalt, forward allocator
    BasicAllocated( size_t size, __Allocator && allocator )
    :
        _BasicMemBlock( NULL, size ),
        allocator_( std::forward<__Allocator>( allocator ) ),
        owns_( true )
    {
//        atomics_print( "BasicAllocated default, forward allocator" << std::endl );
        allocate();
    }

/*
    BasicAllocated( size_t size, __Allocator & allocator )
    :
        _BasicMemBlock( NULL, size ),
        allocator_( allocator ),
        owns_( true )
    {
//        atomics::print( "constructing new BasicAllocated" << std::endl );
        allocate();
    }
*/

    ~BasicAllocated()
    {
//        atomics::print( "destructing BasicAllocated" << std::endl );
        if( owns_ ) deallocate();
    }

    _BasicAllocated & resize( size_t num_items )
    {
        if( num_items > this->size() ) allocate( num_items - this->size() );
        return *this;
    }

    _BasicAllocated & allocate( size_t num_items )
    {
        if( !this->data_ && num_items > 0 ) this->data_ = allocator_.allocate( num_items );
        this->size_ = num_items;

        return *this;
    }

    template<class __OtherData>
    _BasicAllocated & allocate( size_t num_items )
    {
        return allocate( BasicMemBlockConverter<__OtherData, __Data>::size( num_items ) );
    }

    _BasicAllocated & allocateBytes( size_t num_bytes )
    {
        return allocate<char>( num_bytes );
    }

    _BasicAllocated & allocate()
    {
        return allocate( this->size() );
    }

    _BasicAllocated & deallocate( size_t num_items )
    {
        if( this->begin() ) allocator_.deallocate( this->begin(), num_items );
        return *this;
    }

    template<class __OtherData>
    _BasicAllocated & deallocate( size_t num_items )
    {
        return deallocate( BasicMemBlockConverter<__OtherData, __Data>::size( num_items ) );
    }

    _BasicAllocated & deallocateBytes( size_t num_bytes )
    {
        return deallocate<char>( num_bytes );
    }

    _BasicAllocated & deallocate()
    {
        return deallocate( this->size() );
    }

    _BasicAllocated & reallocate( size_t num_items )
    {
        deallocate();
        allocate( num_items );
        return *this;
    }
};

typedef int ExternalAllocator;

template<class __Data = char, class __Allocator = std::allocator<__Data> >
class MemBlock;

template<class __Data>
class MemBlock<__Data, ExternalAllocator> : public BasicMemBlock<__Data>
{
public:
    typedef BasicMemBlock<__Data> _BasicMemBlock;

    typedef __Data _Data;

    MemBlock( __Data * data = NULL, size_t size = 0 )
    :
        _BasicMemBlock( data, size )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << this->size();
        archive.write( reinterpret_cast<char const *>( this->begin() ), this->sizeBytes() );
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> this->size_;
        archive.read( reinterpret_cast<char *>( this->begin() ), this->sizeBytes() );
    }

    size_t packedSize() const
    {
        return this->sizeBytes() + sizeof( this->size() );
    }

    friend std::ostream & operator<<( std::ostream & stream, MemBlock<__Data, void> const & mem_block ){ mem_block.pack( stream ); }
    friend std::istream & operator>>( std::istream & stream, MemBlock<__Data, void> const & mem_block ){ mem_block.unpack( stream ); }
};

namespace archive_adapter
{

template<class __Archive>
struct read
{
    typedef __Archive _Archive;

    template<class... __Args>
    read( _Archive & archive, __Args&&... args ){ archive.read( std::forward<__Args>( args )... ); }
};

template<class __Archive>
struct write
{
    typedef __Archive _Archive;

    template<class... __Args>
    write( _Archive & archive, __Args&&... args ){ archive.write( std::forward<__Args>( args )... ); }
};

template<>
struct read<Poco::BinaryReader>
{
    typedef Poco::BinaryReader _Archive;

    template<class... __Args>
    read( _Archive & archive, __Args&&... args ){ archive.readRaw( std::forward<__Args>( args )... ); }
};

template<>
struct write<Poco::BinaryWriter>
{
    typedef Poco::BinaryWriter _Archive;

    template<class... __Args>
    write( _Archive & archive, __Args&&... args ){ archive.writeRaw( std::forward<__Args>( args )... ); }
};

} // archive_adapter

/*
template<template<class> class __Container1, template<class> class __Container2>
struct ContainerIsSame
{
    static const bool value = false;
};

template<template<class> class __Container>
struct ContainerIsSame<__Container, __Container>
{
    static const bool value = true;
};

template<class __Container1, class __Container2>
struct ContainerIsSame
{
    static bool const value = false;
};

template<class __Container>
struct ContainerIsSame<__Container, __Container>
*/

/*
template<template<class...> class __Container1, template<class...> class __Container2, typename... __Args>
struct ContainerIsSame : std::is_same<__Container1<__Args...>, __Container2<__Args...> >
{
    //
};
*/

template<class __Container1, class __Container2>
struct container_is_same : std::false_type{};

template<template<class> class __Container, class... __Data1, class... __Data2>
struct container_is_same<__Container<__Data1...>, __Container<__Data2...> > : std::true_type{};

template<class __Container1, class __Container2>
struct container_rebind;

template<template<class> class __Container, class... __Args, template<class> class __Rebind, class... __RebindArgs>
struct container_rebind<__Container<__Args...>, __Rebind<__RebindArgs...> >
{
    typedef __Rebind<__Args...> type;
};

template<class __Data, class __Allocator>
class MemBlock : public BasicAllocated<__Data, __Allocator>
{
public:
    typedef MemBlock<__Data, __Allocator> _MemBlock;
    typedef BasicAllocated<__Data, __Allocator> _BasicAllocated;
    typedef typename _BasicAllocated::_BasicMemBlock _BasicMemBlock;

    typedef __Data _Data;
    typedef __Allocator _Allocator;

    // default
    MemBlock( size_t size = 0 )
    :
        _BasicAllocated( size, __Allocator() )
    {
//        atomics_print( "MemBlock default" << std::endl );
    }

    // default, forward allocator
    template<class __OtherAllocator, typename std::enable_if<container_is_same<__Allocator, typename std::remove_reference<__OtherAllocator>::type>::value, int>::type = 0>
    MemBlock( size_t size, __OtherAllocator && allocator )
    :
        _BasicAllocated( size, std::forward<__OtherAllocator>( allocator ) )
    {
//        atomics_print( "MemBlock default, forward allocator" << std::endl );
    }

    // forward-construct __Allocator from one non-__Allocator<T> argument
    template<class __Arg, typename std::enable_if<!container_is_same<__Allocator, typename std::remove_reference<__Arg>::type>::value, int>::type = 0>
    MemBlock( size_t size, __Arg && arg )
    :
        _BasicAllocated( size, __Allocator( std::forward<__Arg>( arg ) ) )
    {
//        atomics_print( "MemBlock default, forward allocator arg" << std::endl );
    }

    // special constructor to forward-construct allocator from two or more args
    template<class... __Args, typename std::enable_if<(sizeof...(__Args) > 1), int>::type = 0>
    MemBlock( size_t size, __Args&&... args )
    :
        _BasicAllocated( size, __Allocator( std::forward<__Args>( args )... ) )
    {
//        atomics_print( "MemBlock default, forward allocator args" << std::endl );
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
//        atomics_print( "packing MemBlock data " << this->begin() << " size " << this->size() << " sizeBytes " << this->sizeBytes() << std::endl );
        archive << this->sizeBytes();
        archive.write( reinterpret_cast<char const *>( this->begin() ), this->sizeBytes() );
//        archive_adapter::write<__Archive>( archive, reinterpret_cast<char const *>( this->begin() ), this->sizeBytes() );
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        size_t size;
        archive >> size;

/*
        if( size > this->sizeBytes() )
        {
            this->allocateBytes( size );
        }
*/
        archive.read( reinterpret_cast<char *>( this->begin() ), this->sizeBytes() );
//        archive_adapter::read<__Archive>( archive, reinterpret_cast<char *>( this->begin() ), this->sizeBytes() );
    }

    size_t packedSize() const
    {
        return this->sizeBytes() + sizeof( this->sizeBytes() );
    }

    template<class __Archive>
    friend __Archive & operator<<( __Archive & archive, _MemBlock const & mem_block ){ mem_block.pack( archive ); return archive; }

    template<class __Archive>
    friend __Archive & operator>>( __Archive & archive, _MemBlock & mem_block ){ mem_block.unpack( archive ); return archive; }

};

} // atomics


#endif // _ATOMICS_MEMBLOCK_H_
