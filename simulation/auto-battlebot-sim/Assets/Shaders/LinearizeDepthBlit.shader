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
    
    Varyings Vert(Attributes input)
    {
        Varyings output;
        output.positionCS = GetFullScreenTriangleVertexPosition(input.vertexID);
        output.texcoord = GetFullScreenTriangleTexCoord(input.vertexID);
        return output;
    }
    
    float4 Frag(Varyings input) : SV_Target
    {
        // Sample depth using HDRP's proper depth access
        float2 uv = input.texcoord;
        
        // Load depth from HDRP's depth buffer
        // LOAD_TEXTURE2D_X samples from the correct slice in XR
        float rawDepth = LoadCameraDepth(uv * _ScreenSize.xy);
        
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
