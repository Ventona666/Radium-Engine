#ifndef RADIUMENGINE_BINDABLEMATERIAL_HPP
#define RADIUMEGNINE_BINDABLEMATERIAL_HPP

#include <Engine/Renderer/RenderQueue/Bindable.hpp>

namespace Ra { namespace Engine { class Material; } }

namespace Ra { namespace Engine {

class BindableMaterial : public Bindable
{
public:
    explicit BindableMaterial(Material* material);
    virtual ~BindableMaterial();

    virtual void bind(ShaderProgram* shader) const override;

    bool operator<(const BindableMaterial& other) const;

private:
    Material* m_material;
};

} // namespace Engine
} // namespace Ra

#endif // RADIUMEGNINE_BINDABLEMATERIAL_HPP
