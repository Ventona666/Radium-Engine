#pragma once
#include <Core/Asset/GeometryData.hpp>

#include <algorithm> //std::transform

namespace Ra {
namespace Core {
namespace Asset {

inline void GeometryData::setName( const std::string& name ) {
    m_name = name;
}

inline GeometryData::GeometryType GeometryData::getType() const {
    return m_type;
}

inline void GeometryData::setType( const GeometryType& type ) {
    m_type = type;
}

inline Transform GeometryData::getFrame() const {
    return m_frame;
}

inline void GeometryData::setFrame( const Transform& frame ) {
    m_frame = frame;
}

inline std::size_t GeometryData::getVerticesSize() const {
    return m_vertexAttribArray.vertices().size();
}

inline const Vector3Array& GeometryData::getVertices() const {
    auto& h = m_vertexAttribArray.vertices();
    return h;
}

inline Vector3Array& GeometryData::getVertices() {
    auto& h = m_vertexAttribArray.verticesWithLock();
    return h;
}

namespace internal {

template <typename InContainer, typename OutContainer>
inline void copyData( const InContainer& input, OutContainer& output ) {
    using OutValueType  = typename OutContainer::value_type;
    using OutScalarType = typename OutValueType::Scalar;
    std::transform( std::begin( input ),
                    std::end( input ),
                    std::back_inserter( output ),
                    []( const typename InContainer::value_type& v ) -> OutValueType {
                        return v.template cast<OutScalarType>();
                    } );
}

} // namespace internal

template <typename Container>
inline void GeometryData::setVertices( const Container& vertexList ) {
    m_vertexAttribArray.setVertices( vertexList );
}

inline Vector2uArray& GeometryData::getEdges() {
    return getAttribDataWithLock<Vector2ui>( "edge" );
}

inline const Vector2uArray& GeometryData::getEdges() const {
    return getAttribData<Vector2ui>( "edge" );
}

template <typename Container>
inline void GeometryData::setEdges( const Container& edgeList ) {
    setAttribData( "edge", edgeList );
}

inline const VectorNuArray& GeometryData::getFaces() const {
    return getAttribData<VectorNui>( "face" );
}

inline VectorNuArray& GeometryData::getFaces() {
    return getAttribDataWithLock<VectorNui>( "face" );
}

template <typename Container>
inline void GeometryData::setFaces( const Container& faceList ) {
    setAttribData( "face", faceList );
}

inline VectorNuArray& GeometryData::getPolyhedra() {
    return getAttribDataWithLock<VectorNui>( "polyhedron" );
}

inline const VectorNuArray& GeometryData::getPolyhedra() const {
    return getAttribData<VectorNui>( "polyhedron" );
    ;
}

template <typename Container>
inline void GeometryData::setPolyhedra( const Container& polyList ) {
    setAttribData( "polyhedron", polyList );
}

inline Vector3Array& GeometryData::getNormals() {
    auto& data = m_vertexAttribArray.normalsWithLock();
    return data;
}

inline const Vector3Array& GeometryData::getNormals() const {
    auto& data = m_vertexAttribArray.normals();
    return data;
}

template <typename Container>
inline void GeometryData::setNormals( const Container& normalList ) {
    m_vertexAttribArray.setNormals( normalList );
}

inline Vector3Array& GeometryData::getTangents() {
    return getAttribDataWithLock<Vector3>( "tangent" );
}

inline const Vector3Array& GeometryData::getTangents() const {
    return getAttribData<Vector3>( "tangent" );
}

template <typename Container>
inline void GeometryData::setTangents( const Container& tangentList ) {
    setAttribData( "tangent", tangentList );
}

inline Vector3Array& GeometryData::getBiTangents() {
    return getAttribDataWithLock<Vector3>( "biTangent" );
}

inline const Vector3Array& GeometryData::getBiTangents() const {
    return getAttribData<Vector3>( "biTangent" );
}

template <typename Container>
inline void GeometryData::setBitangents( const Container& bitangentList ) {
    setAttribData( "biTangent", bitangentList );
}

inline Vector3Array& GeometryData::getTexCoords() {
    return getAttribDataWithLock<Vector3>( "texCoord" );
}

inline const Vector3Array& GeometryData::getTexCoords() const {
    return getAttribData<Vector3>( "texCoord" );
}

template <typename Container>
inline void GeometryData::setTextureCoordinates( const Container& texCoordList ) {
    setAttribData( "texCoord", texCoordList );
}

inline const MaterialData& GeometryData::getMaterial() const {
    return *( m_material.get() );
}

inline void GeometryData::setMaterial( MaterialData* material ) {
    m_material.reset( material );
}

inline bool GeometryData::isPointCloud() const {
    return ( m_type == POINT_CLOUD );
}

inline bool GeometryData::isLineMesh() const {
    return ( m_type == LINE_MESH );
}

inline bool GeometryData::isTriMesh() const {
    return ( m_type == TRI_MESH );
}

inline bool GeometryData::isQuadMesh() const {
    return ( m_type == QUAD_MESH );
}

inline bool GeometryData::isPolyMesh() const {
    return ( m_type == POLY_MESH );
}

inline bool GeometryData::isTetraMesh() const {
    return ( m_type == TETRA_MESH );
}

inline bool GeometryData::isHexMesh() const {
    return ( m_type == HEX_MESH );
}

inline bool GeometryData::hasVertices() const {
    return hasAttribData<Vector3>( "vertex" );
}

inline bool GeometryData::hasEdges() const {
    return hasAttribData<Vector2ui>( "edge" );
}

inline bool GeometryData::hasFaces() const {
    return hasAttribData<VectorNui>( "face" );
}

inline bool GeometryData::hasPolyhedra() const {
    return hasAttribData<Vector2ui>( "polyhedron" );
}

inline bool GeometryData::hasNormals() const {
    return hasAttribData<Vector3>( "normal" );
}

inline bool GeometryData::hasTangents() const {
    return hasAttribData<Vector3>( "tangent" );
}

inline bool GeometryData::hasBiTangents() const {
    return hasAttribData<Vector3>( "biTangent" );
}

inline bool GeometryData::hasTextureCoordinates() const {
    return hasAttribData<Vector3>( "texCoord" );
}

inline bool GeometryData::hasMaterial() const {
    return m_material != nullptr;
}

const Utils::AttribManager& GeometryData::getAttribManager() const {
    return m_vertexAttribArray.vertexAttribs();
}

Utils::AttribManager& GeometryData::getAttribManager() {
    return m_vertexAttribArray.vertexAttribs();
}

template <typename V>
inline VectorArray<V>& GeometryData::getAttribDataWithLock( const std::string& name ) {
    auto h = m_vertexAttribArray.template getAttribHandle<V>( name );
    if ( !m_vertexAttribArray.template isValid( h ) ) {
        h = m_vertexAttribArray.template addAttrib<V>( name );
    }
    auto& attrib = m_vertexAttribArray.template getAttrib( h );
    auto& d      = attrib.getDataWithLock();
    return d;
}

template <typename V>
inline const VectorArray<V>& GeometryData::getAttribData( const std::string& name ) const {
    auto h             = m_vertexAttribArray.template getAttribHandle<V>( name );
    const auto& attrib = m_vertexAttribArray.template getAttrib( h );
    auto& d            = attrib.data();
    return d;
}

template <typename Container>
inline void GeometryData::setAttribData( const std::string& name,
                                         const Container& attribDataList ) {
    Utils::Attrib<Container>& c =
        m_vertexAttribArray.getAttribBase( name )->template cast<Container>();
    auto& v = c.getDataWithLock();
    internal::copyData( attribDataList, v );
    attribDataUnlock( name );
}

template <typename V>
bool GeometryData::hasAttribData( const std::string& name ) const {
    if ( name == "vertex" ) { return !m_vertexAttribArray.vertices().empty(); }
    else if ( name == "normal" ) {
        return !m_vertexAttribArray.normals().empty();
    }
    else {
        auto h = m_vertexAttribArray.template getAttribHandle<V>( name );
        if ( m_vertexAttribArray.template isValid( h ) ) {
            return !m_vertexAttribArray.template getAttrib( h ).data().empty();
        }
    }
    return false;
}

void GeometryData::attribDataUnlock( const std::string& name ) {
    if ( name == "vertex" ) { m_vertexAttribArray.verticesUnlock(); }
    else if ( name == "normal" ) {
        m_vertexAttribArray.normalsUnlock();
    }
    else {
        m_vertexAttribArray.getAttribBase( name )->unlock();
    }
}

} // namespace Asset
} // namespace Core
} // namespace Ra
