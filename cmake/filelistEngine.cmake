set( engine_sources
    Component/Component.cpp
    Component/GeometryComponent.cpp
    Entity/Entity.cpp
    ItemModel/ItemEntry.cpp
    Managers/CameraManager/CameraManager.cpp
    Managers/CameraManager/DefaultCameraManager.cpp
    Managers/ComponentMessenger/ComponentMessenger.cpp
    Managers/EntityManager/EntityManager.cpp
    Managers/LightManager/DefaultLightManager.cpp
    Managers/LightManager/LightManager.cpp
    Managers/SignalManager/SignalManager.cpp
    Managers/SystemDisplay/SystemDisplay.cpp
    RadiumEngine.cpp
    Renderer/Camera/Camera.cpp
    Renderer/Displayable/VolumeObject.cpp
    Renderer/Light/DirLight.cpp
    Renderer/Light/Light.cpp
    Renderer/Light/PointLight.cpp
    Renderer/Light/SpotLight.cpp
    Renderer/Material/BlinnPhongMaterial.cpp
    Renderer/Material/LambertianMaterial.cpp
    Renderer/Material/MaterialConverters.cpp
    Renderer/Material/Material.cpp
    Renderer/Material/PlainMaterial.cpp
    Renderer/Material/SimpleMaterial.cpp
    Renderer/Material/VolumetricMaterial.cpp
    Renderer/Mesh/Mesh.cpp
    Renderer/Renderer.cpp
    Renderer/Renderers/DebugRender.cpp
    Renderer/Renderers/ForwardRenderer.cpp
    Renderer/RenderObject/Primitives/DrawPrimitives.cpp
    Renderer/RenderObject/RenderObject.cpp
    Renderer/RenderObject/RenderObjectManager.cpp
    Renderer/RenderTechnique/RenderParameters.cpp
    Renderer/RenderTechnique/RenderTechnique.cpp
    Renderer/RenderTechnique/ShaderConfigFactory.cpp
    Renderer/RenderTechnique/ShaderConfiguration.cpp
    Renderer/RenderTechnique/ShaderProgram.cpp
    Renderer/RenderTechnique/ShaderProgramManager.cpp
    Renderer/Texture/Texture.cpp
    Renderer/Texture/TextureManager.cpp
    System/GeometrySystem.cpp
    System/System.cpp
    System/TimedSystem.cpp
)

set( engine_headers
    Component/Component.hpp
    Component/GeometryComponent.hpp
    Entity/Entity.hpp
    FrameInfo.hpp
    ItemModel/ItemEntry.hpp
    Managers/CameraManager/CameraManager.hpp
    Managers/CameraManager/CameraStorage.hpp
    Managers/CameraManager/DefaultCameraManager.hpp
    Managers/ComponentMessenger/ComponentMessenger.hpp
    Managers/EntityManager/EntityManager.hpp
    Managers/LightManager/DefaultLightManager.hpp
    Managers/LightManager/LightManager.hpp
    Managers/LightManager/LightStorage.hpp
    Managers/SignalManager/SignalManager.hpp
    Managers/SystemDisplay/SystemDisplay.hpp
    RadiumEngine.hpp
    RaEngine.hpp
    Renderer/Camera/Camera.hpp
    Renderer/Camera/ViewingParameters.hpp
    Renderer/Displayable/DisplayableObject.hpp
    Renderer/Displayable/VolumeObject.hpp
    Renderer/Light/DirLight.hpp
    Renderer/Light/Light.hpp
    Renderer/Light/PointLight.hpp
    Renderer/Light/SpotLight.hpp
    Renderer/Material/BlinnPhongMaterial.hpp
    Renderer/Material/LambertianMaterial.hpp
    Renderer/Material/MaterialConverters.hpp
    Renderer/Material/Material.hpp
    Renderer/Material/PlainMaterial.hpp
    Renderer/Material/SimpleMaterial.hpp
    Renderer/Material/VolumetricMaterial.hpp
    Renderer/Mesh/Mesh.hpp
    Renderer/OpenGL/OpenGL.hpp
    Renderer/Renderer.hpp
    Renderer/Renderers/DebugRender.hpp
    Renderer/Renderers/ForwardRenderer.hpp
    Renderer/RenderObject/Primitives/DrawPrimitives.hpp
    Renderer/RenderObject/RenderObject.hpp
    Renderer/RenderObject/RenderObjectManager.hpp
    Renderer/RenderObject/RenderObjectTypes.hpp
    Renderer/RenderTechnique/RenderParameters.hpp
    Renderer/RenderTechnique/RenderTechnique.hpp
    Renderer/RenderTechnique/ShaderConfigFactory.hpp
    Renderer/RenderTechnique/ShaderConfiguration.hpp
    Renderer/RenderTechnique/ShaderProgram.hpp
    Renderer/RenderTechnique/ShaderProgramManager.hpp
    Renderer/Texture/Texture.hpp
    Renderer/Texture/TextureManager.hpp
    System/CouplingSystem.hpp
    System/GeometrySystem.hpp
    System/System.hpp
    System/TimedSystem.hpp
)

set( engine_inlines
    Component/Component.inl
    Entity/Entity.inl
    ItemModel/ItemEntry.inl
    Managers/ComponentMessenger/ComponentMessenger.inl
    Renderer/Camera/Camera.inl
    Renderer/Displayable/VolumeObject.inl
    Renderer/Light/DirLight.inl
    Renderer/Light/Light.inl
    Renderer/Light/PointLight.inl
    Renderer/Light/SpotLight.inl
    Renderer/Material/BlinnPhongMaterial.inl
    Renderer/Material/Material.inl
    Renderer/Material/SimpleMaterial.inl
    Renderer/Material/VolumetricMaterial.inl
    Renderer/Mesh/Mesh.inl
    Renderer/RenderTechnique/RenderParameters.inl
    Renderer/RenderTechnique/RenderTechnique.inl
)

set(engine_shaders
    2DShaders/Basic2D.vert.glsl
    2DShaders/CircleBrush.frag.glsl
    2DShaders/ComposeOIT.frag.glsl
    2DShaders/DepthDisplay.frag.glsl
    2DShaders/DrawScreen.frag.glsl
    2DShaders/DrawScreenI.frag.glsl
    2DShaders/Hdr2Ldr.frag.glsl
    HdrToLdr/Hdr2Ldr.vert.glsl
    Lights/DefaultLight.glsl
    Lights/DirectionalLight.glsl
    Lights/PointLight.glsl
    Lights/SpotLight.glsl
    Lines/LinesAdjacency.geom.glsl
    Lines/Lines.frag.glsl
    Lines/Lines.geom.glsl
    Lines/Lines.vert.glsl
    Materials/BlinnPhong/BlinnPhong.frag.glsl
    Materials/BlinnPhong/BlinnPhong.glsl
    Materials/BlinnPhong/BlinnPhong.vert.glsl
    Materials/BlinnPhong/DepthAmbientBlinnPhong.frag.glsl
    Materials/BlinnPhong/LitOITBlinnPhong.frag.glsl
    Materials/Lambertian/Lambertian.frag.glsl
    Materials/Lambertian/Lambertian.glsl
    Materials/Lambertian/Lambertian.vert.glsl
    Materials/Lambertian/LambertianZPrepass.frag.glsl
    Materials/Plain/Plain.frag.glsl
    Materials/Plain/Plain.glsl
    Materials/Plain/Plain.vert.glsl
    Materials/Plain/PlainZPrepass.frag.glsl
    Materials/VertexAttribInterface.frag.glsl
    Materials/Volumetric/ComposeVolumeRender.frag.glsl
    Materials/Volumetric/Volumetric.frag.glsl
    Materials/Volumetric/Volumetric.glsl
    Materials/Volumetric/VolumetricOIT.frag.glsl
    Materials/Volumetric/Volumetric.vert.glsl
    Picking/Picking.frag.glsl
    Picking/PickingLinesAdjacency.geom.glsl
    Picking/PickingLines.geom.glsl
    Picking/PickingPoints.geom.glsl
    Picking/PickingTriangles.geom.glsl
    Picking/Picking.vert.glsl
    Points/PointCloud.geom.glsl
    Transform/TransformStructs.glsl
)
