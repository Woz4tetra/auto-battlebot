// HDRP Custom Pass for depth buffer capture
// This pass copies the camera depth buffer to a readable RenderTexture

#if UNITY_HDRP
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace AutoBattlebot.Camera
{
    /// <summary>
    /// HDRP Custom Pass that copies the camera depth buffer to a readable texture.
    /// Automatically finds CameraSimulator and uses its depth target.
    /// </summary>
    [System.Serializable]
    public class HDRPDepthCopyPass : CustomPass
    {
        [Tooltip("Target texture to copy depth into. If empty, will auto-find from CameraSimulator.")]
        public RenderTexture targetDepthTexture;

        private CameraSimulator _cameraSimulator;
        private bool _searchedForSimulator;

        protected override bool executeInSceneView => false;

        protected override void Setup(ScriptableRenderContext renderContext, CommandBuffer cmd)
        {
            name = "Depth Copy Pass";
        }

        protected override void Execute(CustomPassContext ctx)
        {
            // Auto-find CameraSimulator if we don't have a target texture
            if (targetDepthTexture == null)
            {
                if (!_searchedForSimulator)
                {
                    _cameraSimulator = Object.FindFirstObjectByType<CameraSimulator>();
                    _searchedForSimulator = true;
                    
                    if (_cameraSimulator != null)
                    {
                        Debug.Log("[HDRPDepthCopyPass] Found CameraSimulator, will use its depth target");
                    }
                    else
                    {
                        Debug.LogWarning("[HDRPDepthCopyPass] No CameraSimulator found");
                    }
                }

                if (_cameraSimulator != null)
                {
                    targetDepthTexture = _cameraSimulator.DepthTarget;
                }
            }

            if (targetDepthTexture == null)
            {
                return;
            }

            // Copy the depth buffer
            RTHandle depthBuffer = ctx.cameraDepthBuffer;
            
            if (depthBuffer != null && depthBuffer.rt != null)
            {
                ctx.cmd.Blit(depthBuffer.rt, targetDepthTexture);
            }
            else
            {
                // Fallback: try shader global
                var depthTexture = Shader.GetGlobalTexture("_CameraDepthTexture");
                if (depthTexture != null)
                {
                    ctx.cmd.Blit(depthTexture, targetDepthTexture);
                }
            }
        }

        protected override void Cleanup()
        {
            _cameraSimulator = null;
            _searchedForSimulator = false;
        }
    }
}
#endif
