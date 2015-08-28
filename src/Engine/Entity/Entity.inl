#include <Engine/Entity/Entity.hpp>
#include <Core/Log/Log.hpp>

namespace Ra
{

    inline const std::string& Engine::Entity::getName() const
    {
        return m_name;
    }

    inline void Engine::Entity::rename( const std::string& name )
    {
        m_name = name;
    }

    inline void Engine::Entity::setTransform( const Core::Transform& transform )
    {
        if ( m_transformChanged )
        {
            LOG( logWARNING ) << "This entity transform has already been set during this frame, ignored.";
            return;
        }
        m_transformChanged = true;
        m_doubleBufferedTransform = transform;
    }

    inline void Engine::Entity::setTransform( const Core::Matrix4& transform )
    {
        setTransform( Core::Transform( transform ) );
    }

    inline Core::Transform Engine::Entity::getTransform() const
    {
        std::lock_guard<std::mutex> lock( m_transformMutex );
        return m_transform;
    }

    inline Core::Matrix4 Engine::Entity::getTransformAsMatrix() const
    {
        std::lock_guard<std::mutex> lock( m_transformMutex );
        return m_transform.matrix();
    }

} // namespace Ra
