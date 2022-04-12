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
    return m_vertex.size();
}

inline const Vector3Array& GeometryData::getVertices() const {
    return m_vertex;
}

inline Vector3Array& GeometryData::getVertices() {
    return m_vertex;
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
    internal::copyData( vertexList, m_vertex );
}

inline Vector2uArray& GeometryData::getEdges() {
    return m_edge;
}

inline const Vector2uArray& GeometryData::getEdges() const {
    return m_edge;
}

template <typename Container>
inline void GeometryData::setEdges( const Container& edgeList ) {
    internal::copyData( edgeList, m_edge );
}

inline const VectorNuArray& GeometryData::getFaces() const {
    return m_faces;
}

inline VectorNuArray& GeometryData::getFaces() {
    return m_faces;
}

template <typename Container>
inline void GeometryData::setFaces( const Container& faceList ) {
    internal::copyData( faceList, m_faces );
}

inline VectorNuArray& GeometryData::getPolyhedra() {
    return m_polyhedron;
}

inline const VectorNuArray& GeometryData::getPolyhedra() const {
    return m_polyhedron;
}

template <typename Container>
inline void GeometryData::setPolyhedra( const Container& polyList ) {
    internal::copyData( polyList, m_polyhedron );
}

inline Vector3Array& GeometryData::getNormals() {
    auto h = m_vertexAttribs.findAttrib<Vector3>( "normal" );
    if ( !m_vertexAttribs.isValid( h ) ) { h = m_vertexAttribs.addAttrib<Vector3>( "normal" ); }
    auto& attrib = m_vertexAttribs.getAttrib( h );
    auto& normal = attrib.getDataWithLock();
    attribPtr    = &attrib;
    return normal;
}

inline void GeometryData::unlockData() {
    attribPtr->unlock();
    attribPtr = nullptr;
}

inline const Vector3Array& GeometryData::getNormals() const {
    auto handler = m_vertexAttribs.findAttrib<Vector3>( "normal" );
    auto& normal = m_vertexAttribs.getAttrib( handler ).data();
    return normal;
}

template <typename Container>
inline void GeometryData::setNormals( const Container& normalList ) {
    auto& attrib = m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "normal" ) );
    auto& normal = attrib.getDataWithLock();
    internal::copyData( normalList, normal );
    attrib.unlock();
}

inline Vector3Array& GeometryData::getTangents() {
    auto h = m_vertexAttribs.findAttrib<Vector3>( "tangent" );
    if ( !m_vertexAttribs.isValid( h ) ) { h = m_vertexAttribs.addAttrib<Vector3>( "tangent" ); }
    auto& attrib  = m_vertexAttribs.getAttrib( h );
    auto& tangent = attrib.getDataWithLock();
    attribPtr     = &attrib;
    return tangent;
}

inline const Vector3Array& GeometryData::getTangents() const {
    auto attriHandler = m_vertexAttribs.findAttrib<Vector3>( "tangent" );
    auto& tangent     = m_vertexAttribs.getAttrib( attriHandler ).data();
    return tangent;
}

template <typename Container>
inline void GeometryData::setTangents( const Container& tangentList ) {
    auto& attrib  = m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "normal" ) );
    auto& tangent = attrib.getDataWithLock();
    internal::copyData( tangentList, tangent );
    attrib.unlock();
}

inline Vector3Array& GeometryData::getBiTangents() {
    auto h = m_vertexAttribs.findAttrib<Vector3>( "biTangent" );
    if ( !m_vertexAttribs.isValid( h ) ) { h = m_vertexAttribs.addAttrib<Vector3>( "biTangent" ); }
    auto& attrib    = m_vertexAttribs.getAttrib( h );
    auto& biTangent = attrib.getDataWithLock();
    attribPtr       = &attrib;
    return biTangent;
}

inline const Vector3Array& GeometryData::getBiTangents() const {
    auto attriHandler = m_vertexAttribs.findAttrib<Vector3>( "biTangent" );
    auto& biTangent   = m_vertexAttribs.getAttrib( attriHandler ).data();
    return biTangent;
}

template <typename Container>
inline void GeometryData::setBitangents( const Container& bitangentList ) {
    auto& attrib = m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "biTangent" ) );
    auto& biTangent = attrib.getDataWithLock();
    internal::copyData( bitangentList, biTangent );
    attrib.unlock();
}

inline Vector3Array& GeometryData::getTexCoords() {
    auto h = m_vertexAttribs.findAttrib<Vector3>( "texCoord" );
    if ( !m_vertexAttribs.isValid( h ) ) { h = m_vertexAttribs.addAttrib<Vector3>( "texCoord" ); }
    auto& attrib   = m_vertexAttribs.getAttrib( h );
    auto& texCoord = attrib.getDataWithLock();
    attribPtr      = &attrib;
    return texCoord;
}

inline const Vector3Array& GeometryData::getTexCoords() const {
    auto attriHandler = m_vertexAttribs.findAttrib<Vector3>( "texCoord" );
    auto& texCoord    = m_vertexAttribs.getAttrib( attriHandler ).data();
    return texCoord;
}

template <typename Container>
inline void GeometryData::setTextureCoordinates( const Container& texCoordList ) {
    auto& attrib   = m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "texCoord" ) );
    auto& texCoord = attrib.getDataWithLock();
    internal::copyData( texCoordList, texCoord );
    attrib.unlock();
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
    return !m_vertex.empty();
}

inline bool GeometryData::hasEdges() const {
    return !m_edge.empty();
}

inline bool GeometryData::hasFaces() const {
    return !m_faces.empty();
}

inline bool GeometryData::hasPolyhedra() const {
    return !m_polyhedron.empty();
}

inline bool GeometryData::hasNormals() const {
    return !m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "normal" ) )
                .data()
                .empty();
}

inline bool GeometryData::hasTangents() const {
    return !m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "tangent" ) )
                .data()
                .empty();
}

inline bool GeometryData::hasBiTangents() const {
    return !m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "biTangent" ) )
                .data()
                .empty();
}

inline bool GeometryData::hasTextureCoordinates() const {
    return !m_vertexAttribs.getAttrib( m_vertexAttribs.findAttrib<Vector3>( "texCoord" ) )
                .data()
                .empty();
}

inline bool GeometryData::hasMaterial() const {
    return m_material != nullptr;
}

const Utils::AttribManager& GeometryData::getAttribManager() const {
    return m_vertexAttribs;
}

Utils::AttribManager& GeometryData::getAttribManager() {
    return m_vertexAttribs;
}

} // namespace Asset
} // namespace Core
} // namespace Ra
