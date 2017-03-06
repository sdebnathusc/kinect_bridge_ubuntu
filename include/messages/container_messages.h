#ifndef _MESSAGES_CONTAINERMESSAGES_H_
#define _MESSAGES_CONTAINERMESSAGES_H_

#include <tuple>
#include <vector>
#include <array>
#include <memory>

#include <messages/serializable_message.h>

class RecursiveMessageHeader : public MessageHeader
{
public:
    uint32_t payload_id_;

    RecursiveMessageHeader( uint32_t const & payload_id = 0 )
    :
        payload_id_( payload_id )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << payload_id_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> payload_id_;
    }
};

template<class __Base>
class RecursiveMessage : public RecursiveMessageHeader
{
public:
    RecursiveMessage()
    :
        RecursiveMessageHeader( __Base::ID() )
    {
        //
    }

    DECLARE_MESSAGE_INFO( RecursiveMessage )
};

class VectorMessageHeader : public RecursiveMessageHeader
{
public:
    uint32_t size_;

    VectorMessageHeader( uint32_t payload_id = 0, uint32_t size = 0 )
    :
        RecursiveMessageHeader( payload_id ),
        size_( size )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << size_;
        RecursiveMessageHeader::pack( archive );

//        std::cout << "packed VectorMessageHeader size: " << size_ << " type: " << this->payload_type_ << std::endl;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> size_;
        RecursiveMessageHeader::unpack( archive );

//        std::cout << "unpacked VectorMessageHeader size: " << size_ << " type: " << this->payload_type_ << std::endl;
    }
};

template<class __Payload>
class VectorMessage : public SerializableMessageInterface<VectorMessageHeader, std::vector<__Payload> >
{
public:
    typedef SerializableMessageInterface<VectorMessageHeader, std::vector<__Payload> > _Message;

    VectorMessage( uint32_t size = 0 )
    :
        _Message( VectorMessageHeader( __Payload::ID(), size ), std::vector<__Payload>( size ) )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        this->header_.size_ = this->payload_.size();

        this->header_.pack( archive );

        for( auto payloads_it = this->payload_.begin(); payloads_it != this->payload_.end(); ++payloads_it )
        {
            payloads_it->pack( archive );
        }
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        this->header_.unpack( archive );

        this->payload_.resize( this->header_.size_ );

        for( auto payloads_it = this->payload_.begin(); payloads_it != this->payload_.end(); ++payloads_it )
        {
            payloads_it->unpack( archive );
        }
    }

    size_t size() const
    {
        return this->payload_.size();
    }

    void push_back( __Payload && payload )
    {
        this->payload_.push_back( std::forward<__Payload>( payload ) );
    }

    __Payload & at( size_t index )
    {
        return this->payload_.at( index );
    }

    __Payload const & at( size_t index ) const
    {
        return this->payload_.at( index );
    }

    __Payload & operator[]( size_t index )
    {
        return at( index );
    }

    __Payload const & operator[]( size_t index ) const
    {
        return at( index );
    }

    void resize( size_t const & size )
    {
        this->payload_.resize( size );
    }

    DECLARE_MESSAGE_INFO( VectorMessage )
};

class ArrayMessageHeader : public RecursiveMessageHeader
{
public:
    uint32_t size_;

    ArrayMessageHeader( uint32_t payload_id = 0, uint32_t size = 0 )
    :
        RecursiveMessageHeader( payload_id ),
        size_( size )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << size_;
        RecursiveMessageHeader::pack( archive );

//        std::cout << "packed ArrayMessageHeader size: " << size_ << " type: " << this->payload_type_ << std::endl;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> size_;
        RecursiveMessageHeader::unpack( archive );

//        std::cout << "unpacked ArrayMessageHeader size: " << size_ << " type: " << this->payload_type_ << std::endl;
    }
};

template<class __Data, int __Dim>
class ArrayHelper
{
public:
    template<class __Iterator>
    ArrayHelper( __Iterator array_it )
    {
        //
    }
};

template<class __Data>
class ArrayHelper<__Data, 7>
{
public:
    __Data & x;
    __Data & y;
    __Data & z;

    __Data & quat_x;
    __Data & quat_y;
    __Data & quat_z;
    __Data & quat_w;

    template<class __Iterator>
    ArrayHelper( __Iterator array_it )
    :
        x( *array_it++ ),
        y( *array_it++ ),
        z( *array_it++ ),
        quat_x( *array_it++ ),
        quat_y( *array_it++ ),
        quat_z( *array_it++ ),
        quat_w( *array_it++ )
    {
        //
    }
};

template<class __Data>
class ArrayHelper<__Data, 6>
{
public:
    __Data & x;
    __Data & y;
    __Data & z;

    __Data & roll;
    __Data & pitch;
    __Data & yaw;

    template<class __Iterator>
    ArrayHelper( __Iterator array_it )
    :
        x( *array_it++ ),
        y( *array_it++ ),
        z( *array_it++ ),
        roll( *array_it++ ),
        pitch( *array_it++ ),
        yaw( *array_it++ )
    {
        //
    }
};

template<class __Data>
class ArrayHelper<__Data, 4> : public ArrayHelper<__Data, 4-1>
{
public:
    __Data & w;

    template<class __Iterator>
    ArrayHelper( __Iterator array_it )
    :
        ArrayHelper<__Data, 4-1>( array_it ),
        w( *(array_it += 3) )
    {
        //
    }
};

template<class __Data>
class ArrayHelper<__Data, 3> : public ArrayHelper<__Data, 3-1>
{
public:
    __Data & z;
    __Data & theta;

    __Data & yaw;

    template<class __Iterator>
    ArrayHelper( __Iterator array_it )
    :
        ArrayHelper<__Data, 3-1>( array_it ),
        z( *(array_it += 2) ),
        theta( *array_it ),
        yaw( *array_it++ )
    {
        //
    }
};

template<class __Data>
class ArrayHelper<__Data, 2>
{
public:
    __Data & x;
    __Data & roll;

    __Data & y;
    __Data & pitch;

    template<class __Iterator>
    ArrayHelper( __Iterator array_it )
    :
        x( *array_it ),
        roll( *array_it++ ),
        y( *array_it ),
        pitch( *array_it++ )
    {
        //
    }
};

// note: this message makes a huge assumption, which is that the header of each stored message is identical; if this is not the case, use VectorMessage instead
template<class __Payload, uint32_t __Dim>
class ArrayMessage : public SerializableMessageInterface<ArrayMessageHeader, std::array<__Payload, __Dim> >, public ArrayHelper<__Payload, __Dim>
{
public:
    typedef SerializableMessageInterface<ArrayMessageHeader, std::array<__Payload, __Dim> > _Message;

    template<class... __Args>
    ArrayMessage( __Args&&... args )
    :
        _Message( ArrayMessageHeader( __Payload::ID(), __Dim ), std::array<__Payload, __Dim>( std::forward<__Args>( args )... ) ),
        ArrayHelper<__Payload, __Dim>( this->payload_.begin() )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        auto & header = this->header_;
        auto & payload = this->payload_;

        header.size_ = payload.size();

        header.pack( archive );

        // pack only the first message header; we assume the rest are identical
        payload.front().packHeader( archive );

        // now pack all the payloads
        for( auto payloads_it = this->payload_.begin(); payloads_it != this->payload_.end(); ++payloads_it )
        {
            payloads_it->packPayload( archive );
        }
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        auto & header = this->header_;
        auto & payload = this->payload_;

        header.unpack( archive );

        // unpack only the first message header; we assume the rest are identical
        payload.front().unpackHeader( archive );

        // now unpack all the payloads and copy the first header to each
        for( auto payloads_it = this->payload_.begin(); payloads_it != this->payload_.end(); ++payloads_it )
        {
            payloads_it->header_ = payload.front().header_;
            payloads_it->unpackPayload( archive );
        }
    }

    size_t size() const
    {
        return this->payload_.size();
    }

    __Payload & at( size_t index )
    {
        return this->payload_.at( index );
    }

    __Payload const & at( size_t index ) const
    {
        return this->payload_.at( index );
    }

    __Payload & operator[]( size_t index )
    {
        return at( index );
    }

    __Payload const & operator[]( size_t index ) const
    {
        return at( index );
    }

    DECLARE_MESSAGE_INFO( ArrayMessage )
};

template<class __Primary, class... __Others>
class TupleMessage
{
public:
    typedef std::tuple<__Primary, __Others...> _Types;

    _Types types_;

    TupleMessage()
    {
        //
    }

    template<class... __Args>
    TupleMessage( __Args&&... args )
    :
        types_( std::make_tuple( std::forward<__Args>( args )... ) )
    {
        //
    }

    template<class __Archive, uint32_t __Index, typename std::enable_if<(__Index==0), int>::type = 0>
    void packImpl( __Archive & archive )
    {
        //
    }

    template<class __Archive, uint32_t __Index, typename std::enable_if<(__Index>0), int>::type = 0>
    void packImpl( __Archive & archive )
    {
        auto & message = std::get<__Index - 1>( types_ );
        archive << message.ID();
        message.pack( archive );

        packImpl<__Archive, __Index - 1>( archive );
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
//        archive << name();
        archive << numTypes();

/*
        archive << __Primary::name();
        __Primary::pack( archive );
*/

        packImpl<__Archive, std::tuple_size<_Types>::value>( archive );
    }

/*
    template<class __Archive, uint32_t __Index>
    void packAsImpl( __Archive & archive )
    {
        //
    }

    template<class __Archive, uint32_t __Index, class __Head, class... __Tail>
    void packAsImpl( __Archive & archive )
    {
        auto & message = std::get<__Index>( types_ );
        archive << static_cast<__Head>( message ).name();
        message.packAs<__Head>( archive );

        packAsImpl<__Index + 1, __Tail...>( archive );
    }

    template<class... __Bases, class __Archive>
    void packAs( __Archive & archive )
    {
        archive << sizeof...( __Bases );

        packAsImpl<0, __Bases...>( archive );
    }
*/

    template<class __Archive, uint32_t __Index, typename std::enable_if<(__Index==0), int>::type = 0>
    void unpackImpl( __Archive & archive )
    {
        //
    }

    template<class __Archive, uint32_t __Index, typename std::enable_if<(__Index>0), int>::type = 0>
    void unpackImpl( __Archive & archive )
    {
        uint32_t id;
        archive >> id;

        auto & message = std::get<__Index - 1>( types_ );
        message.unpack( archive );

        unpackImpl<__Archive, __Index - 1>( archive );
    }

    uint32_t numTypes() const
    {
        return 1 + sizeof...( __Others );
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
//        std::string name;
//        archive >> name;

        uint32_t num_types;
        archive >> num_types;

/*
        archive >> primary_name;
        __Primary::unpack( archive );
*/

        unpackImpl<__Archive, std::tuple_size<_Types>::value>( archive );
    }

/*
    template<class __Archive, uint32_t __Index>
    void unpackAsImpl( __Archive & archive )
    {
        //
    }

    template<class __Archive, uint32_t __Index, class __Head, class... __Tail>
    void unpackAsImpl( __Archive & archive )
    {
        std::string name;
        archive >> name;

        auto & message = std::get<__Index>( types_ );
        message.unpackAs<__Head>( archive );

        unpackAsImpl<__Index + 1, __Tail...>( archive );
    }

    template<class... __Bases, class __Archive>
    void unpackAs( __Archive & archive )
    {
        uint32_t num_types;
        archive >> num_types;

        unpackAsImpl<0, __Bases...>( archive );
    }
*/

    DECLARE_MESSAGE_INFO( TupleMessage )
};

template<class __Primary, class... __Others>
class MultiMessage : public __Primary, public __Others...
{
public:
    typedef __Primary _Primary;
    typedef std::tuple<__Primary, __Others...> _Types;

    typedef std::shared_ptr<MultiMessage<__Primary, __Others...> > _Ptr;

    template<class... __Args>
    MultiMessage( __Args&&... args )
    :
        __Primary( std::forward<__Args>( args )... )
    {
        //
    }

    template<class __Archive>
    void packImpl( __Archive & archive )
    {
        //
    }

    template<class __Archive, class __Head, class... __Tail>
    void packImpl( __Archive & archive )
    {
//        archive << __Head::name();
        __Head::pack( archive );

//        packImpl<__Archive, __Tail...>( archive );
        packImpl<__Archive, __Tail...>( archive );
    }

/*
    template<class __Derived, class __Archive>
    void packDerived( __Archive & archive )
    {
        __Derived::pack( archive );
    }
*/

    template<class __Archive>
    void pack( __Archive & archive )
    {
//        packImpl<__Archive, __Primary, __Others...>( archive );
        packImpl<__Archive, __Primary, __Others...>( archive );
    }

    template<class... __Bases, class __Archive>
    void packAs( __Archive & archive )
    {
        packImpl<__Archive, __Bases...>( archive );
    }

    template<class __Archive>
    void unpackImpl( __Archive & archive )
    {
        //
    }

    template<class __Archive, class __Head, class... __Tail>
    void unpackImpl( __Archive & archive )
    {
//        std::string name;
//        archive >> name;

        __Head::unpack( archive );

        unpackImpl<__Archive, __Tail...>( archive );
//        unpackImpl<__Tail...>( archive );
    }

    uint32_t numTypes()
    {
        return 1 + sizeof...( __Others );
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {

        unpackImpl<__Archive, __Primary, __Others...>( archive );
//        unpackImpl<__Primary, __Others...>( archive );
    }

    template<class... __Bases, class __Archive>
    void unpackAs( __Archive & archive )
    {
        unpackImpl<__Bases...>( archive );
    }

    template<class __Component>
    __Component & getComponent()
    {
        return *static_cast<__Component *>( this );
    }

    template<class __Component>
    __Component const & getComponent() const
    {
        return *static_cast<__Component const *>( this );
    }

    __Primary & getPrimary()
    {
        return getComponent<__Primary>();
    }

    __Primary const & getPrimary() const
    {
        return getComponent<__Primary>();
    }

    DECLARE_MESSAGE_INFO( MultiMessage )
};

#endif // _MESSAGES_CONTAINERMESSAGES_H_
