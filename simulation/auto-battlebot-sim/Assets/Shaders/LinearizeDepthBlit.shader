// Auto-Battlebot Simulation System
// LinearizeDepthBlit.shader - Fragment shader version for HDRP depth linearization
//
// This is an alternative to the compute shader that uses HDRP's proper depth access.

Shader "AutoBattlebot/LinearizeDepthBlit"
{
    HLSLINCLUDE
    
    #pragma target 4.5
    #pragma only_renderers d3d11 playstation xboxone xboxseries vulkan metal switch
    
    #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"
    #include "Packages/com.unity.render-pipelines.high-definition/Runtime/ShaderLibrary/ShaderVariables.hlsl"
    
    struct Attributes
    {
        uint vertexID : SV_VertexID;
    };
    
    struct Varyings
    {
        float4 positionCS : SV_POSITION;
        float2 texcoord : TEXCOORD0;
    };
    
    float4 _DepthRange;  // x = near, y = far
    float _InvalidDepth;
    float4 _DepthBufferSize;  // x = width, y = height, z = 1/width, w = 1/height
    
    Varyings Vert(Attributes input)
    {
        Varyings output;
        output.positionCS = GetFullScreenTriangleVertexPosition(input.vertexID);
        output.texcoord = GetFullScreenTriangleTexCoord(input.vertexID);
        return output;
    }
    
    // Debug mode: 0 = normal, 1 = constant output, 2 = UV gradient, 3 = raw depth
    float _DebugMode;
    
    float4 Frag(Varyings input) : SV_Target
    {
        float2 uv = input.texcoord;
        
        // Debug mode 1: Write constant value to verify pipeline
        if (_DebugMode >= 0.5 && _DebugMode < 1.5)
        {
            return float4(5.0, 0, 0, 1);  // Constant 5 meters
        }
        
        // Debug mode 2: Write UV gradient to verify texture coordinates
        if (_DebugMode >= 1.5 && _DebugMode < 2.5)
        {
            return float4(uv.x * 10.0, 0, 0, 1);  // 0-10 gradient
        }
        
        // Sample depth using HDRP's proper depth access
        // Load depth from HDRP's depth buffer
        // Use _DepthBufferSize instead of _ScreenSize to handle resolution mismatches
        // _DepthBufferSize is set from C# to the actual camera depth buffer dimensions
        float2 pixelCoord = uv * _DepthBufferSize.xy;
        float rawDepth = LoadCameraDepth(pixelCoord);
        
        // Debug mode 3: Output raw depth value
        if (_DebugMode >= 2.5 && _DebugMode < 3.5)
        {
            return float4(rawDepth, 0, 0, 1);
        }
        
        // Handle invalid depth (sky/far plane)
        // In Unity's reversed Z-buffer, 0 = far plane (sky)
        if (rawDepth <= 0.0)
        {
            return float4(_InvalidDepth, 0, 0, 1);
        }
        
        // Convert to linear depth in meters using HDRP's LinearEyeDepth
        float linearDepth = LinearEyeDepth(rawDepth, _ZBufferParams);
        
        // Clamp to valid range
        float nearClip = _DepthRange.x;
        float farClip = _DepthRange.y;
        
        if (linearDepth < nearClip)
        {
            linearDepth = nearClip;
        }
        else if (linearDepth > farClip)
        {
            linearDepth = _InvalidDepth;
        }
        
        return float4(linearDepth, 0, 0, 1);
    }
    
    ENDHLSL
    
    SubShader
    {
        Tags { "RenderPipeline" = "HDRenderPipeline" }
        
        Pass
        {
            Name "LinearizeDepth"
            
            ZWrite Off
            ZTest Always
            Blend Off
            Cull Off
            
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            ENDHLSL
        }
    }
    
    Fallback Off
}
